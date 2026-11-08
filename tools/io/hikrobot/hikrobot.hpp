#pragma once
#include "MvCameraControl.h"
#include "tools/common/image.hpp"
#include "tools/common/type_common.hpp"
#include "tools/io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/scheduler/node.hpp"
#include "tools/scheduler/scheduler.hpp"
#include "tools/utils.hpp"
#include <array>
#include <atomic>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <utility>
namespace dts_vision::tools::io {

class HikCamera: public Camera {
public:
    struct Frame {
        MV_FRAME_OUT out_frame;
        TimePoint timestamp;
    };
    HikCamera(const YAML::Node& config, Scheduler& scheduler): Camera(&scheduler) {
        config_ = config;
        running_ = true;
    }
    void init() override {
        load(config_);
    }
    ~HikCamera() {
        stop();
    }
    void load(const YAML::Node& config);
    void apply_config(const YAML::Node& config);
    void stop() override;
    void restart() override;
    bool is_running() const override {
        return running_;
    }
    ImageFrame read() override;
    bool start_capture() override;

    double get_ExposureTime() const override {
        return ExposureTime_val;
    }
    // 只会被采集线程调用, 见 Camera::flush_exposure_request 上面的注释。
    void write_exposure_time(double exposure_time) override {
        ExposureTime_val = HikSetFloatRange(camera_handle_, "ExposureTime", exposure_time);
    }
    bool stream_faulted() const override {
        return stream_faulted_.load(std::memory_order_acquire);
    }
    // 用相机自己的帧号统计静默丢帧。SDK 只在缓存列表溢出时报一次 0x4002, 不会告诉你
    // 丢了几帧, 而丢帧对下游是不可见的: 时间戳取在出队时刻, 看起来一切正常。
    void account_dropped_frames(const MV_FRAME_OUT_INFO_EX& info);

    const std::unordered_map<MvGvspPixelType, int> CVT_MAP_BGR = {
        { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB },
        { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB },
        { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB },
        { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB },
        { PixelType_Gvsp_RGB8_Packed, -1 },
        { PixelType_Gvsp_Mono8, cv::COLOR_GRAY2RGB },
    };

    const std::unordered_map<MvGvspPixelType, int> CVT_MAP_RGB = {
        { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR },
        { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR },
        { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR },
        { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR },
        { PixelType_Gvsp_RGB8_Packed, cv::COLOR_RGB2BGR },
        { PixelType_Gvsp_Mono8, cv::COLOR_GRAY2BGR },
    };

    const std::unordered_map<MvGvspPixelType, int> CVT_MAP_GRAY = {
        { PixelType_Gvsp_BayerGR8, -1 },
        { PixelType_Gvsp_BayerRG8, -1 },
        { PixelType_Gvsp_BayerGB8, -1 },
        { PixelType_Gvsp_BayerBG8, -1 },
        { PixelType_Gvsp_RGB8_Packed, cv::COLOR_RGB2GRAY },
        { PixelType_Gvsp_Mono8, -1 },
    };
    const std::unordered_map<MvGvspPixelType, int>& get_cvt_map() {
        static const std::array details { CVT_MAP_BGR, CVT_MAP_GRAY, CVT_MAP_RGB };
        return details[std::to_underlying(target_format_)];
    }
    ImageFrame to_image_frame(Frame& f);

    bool initialize_camera(const std::string& target_sn);
    // StopGrabbing + CloseDevice + DestroyHandle, 幂等。注意 MV_CC_DestroyHandle 的
    // 签名是 MV_CC_DestroyHandle(IN void* handle), 要传句柄本身; 以前这里传的是
    // &camera_handle_, 等于交了个野句柄进去, 返回 MV_E_HANDLE, 真句柄从未被销毁。
    void teardown_handle();
    void register_exception_callbacks();
    // SDK 回调里不允许调用 StopGrabbing / CloseDevice / DestroyHandle, 只能置标志。
    static void __stdcall on_stream_exception(MV_CC_STREAM_EXCEPTION_INFO* info, void* user);
    static void __stdcall on_device_exception(unsigned int msg_type, void* user);

    // 返回实际下发的(被相机能力范围 clamp 过的)值, 失败时返回原始请求值。
    inline double HikSetFloatRange(void* camera_handle, const char* param, double val) {
        MVCC_FLOATVALUE fv {};
        int s = MV_CC_GetFloatValue(camera_handle, param, &fv);
        if (s != MV_OK) {
            DTS_ERROR("hik_camera: Failed to get {} range, status={}", param, s);
            return val;
        }

        double c = std::clamp(val, (double)fv.fMin, (double)fv.fMax);
        int r = MV_CC_SetFloatValue(camera_handle, param, c);
        if (r == MV_OK) {
            DTS_INFO("hik_camera: {} set to {}", param, c);
            return c;
        }
        DTS_ERROR("hik_camera: Failed to set {}, status={}", param, r);
        return val;
    }
    inline void HikSetIntRange(void* camera_handle, const char* param, int64_t val) {
        MVCC_INTVALUE iv {};
        int s = MV_CC_GetIntValue(camera_handle, param, &iv);
        if (s != MV_OK) {
            DTS_ERROR("hik_camera: Failed to get {} range, status={}", param, s);
            return;
        }

        int64_t c = std::clamp(val, static_cast<int64_t>(iv.nMin), static_cast<int64_t>(iv.nMax));
        int r = MV_CC_SetIntValue(camera_handle, param, c);
        if (r == MV_OK) {
            DTS_INFO("hik_camera: {} set to {}", param, c);
        } else {
            DTS_ERROR("hik_camera: Failed to set {}, status={}", param, r);
        }
    }
    inline void HikSetBool(void* camera_handle, const char* param, bool val) {
        int r = MV_CC_SetBoolValue(camera_handle, param, val);
        if (r == MV_OK) {
            DTS_INFO("hik_camera: {} set to {}", param, (val ? 1 : 0));
        } else {
            DTS_ERROR("hik_camera: Failed to set {}, status={}", param, r);
        }
    }
    inline void HikSetEnumStr(void* camera_handle, const char* param, const std::string& val) {
        int r = MV_CC_SetEnumValueByString(camera_handle, param, val.c_str());
        if (r == MV_OK) {
            DTS_INFO("hik_camera: {} set to {}", param, val);
        } else {
            DTS_ERROR("hik_camera: Failed to set {}, status={}", param, r);
        }
    }

    template<typename T>
    inline void HikSetRangeDispatch(void* camera_handle, const char* param, const T& val) {
        if constexpr (std::is_same_v<T, bool>) {
            HikSetBool(camera_handle, param, val);

        } else if constexpr (std::is_integral_v<T>) {
            HikSetIntRange(camera_handle, param, static_cast<int64_t>(val));

        } else if constexpr (std::is_floating_point_v<T>) {
            HikSetFloatRange(camera_handle, param, static_cast<double>(val));

        } else if constexpr (std::is_same_v<T, std::string>) {
            HikSetEnumStr(camera_handle, param, val);

        } else {
            static_assert(!sizeof(T), "Unsupported HikCamera param type");
        }
    }

#define HIK_GEN_MEMBER_GET_SET(type, camera_handle, param) \
    type param##_val {}; \
    inline void set_##param(type v) { \
        param##_val = v; \
        HikSetRangeDispatch(camera_handle, #param, param##_val); \
    } \
    inline type get_##param() const { \
        return param##_val; \
    }
    HIK_GEN_MEMBER_GET_SET(std::string, camera_handle_, PixelFormat)
    HIK_GEN_MEMBER_GET_SET(std::string, camera_handle_, ADCBitDepth)
    HIK_GEN_MEMBER_GET_SET(std::string, camera_handle_, TriggerMode)
    HIK_GEN_MEMBER_GET_SET(std::string, camera_handle_, TriggerSource)
    HIK_GEN_MEMBER_GET_SET(std::string, camera_handle_, TriggerActivation)
    HIK_GEN_MEMBER_GET_SET(bool, camera_handle_, ReverseX)
    HIK_GEN_MEMBER_GET_SET(bool, camera_handle_, ReverseY)
    HIK_GEN_MEMBER_GET_SET(int, camera_handle_, Width)
    HIK_GEN_MEMBER_GET_SET(int, camera_handle_, Height)
    HIK_GEN_MEMBER_GET_SET(int, camera_handle_, OffsetX)
    HIK_GEN_MEMBER_GET_SET(int, camera_handle_, OffsetY)
    HIK_GEN_MEMBER_GET_SET(bool, camera_handle_, AcquisitionFrameRateEnable)
    HIK_GEN_MEMBER_GET_SET(double, camera_handle_, AcquisitionFrameRate)
    HIK_GEN_MEMBER_GET_SET(double, camera_handle_, Gain)
    HIK_GEN_MEMBER_GET_SET(double, camera_handle_, Gamma)
    // ExposureTime 不走这个宏: 它是唯一会被其它线程请求修改的参数, 下发必须回到
    // 采集线程, 所以只保留缓存值, 入口是 Camera::set_ExposureTime。
    double ExposureTime_val {};

    void* camera_handle_ = nullptr;
    std::string target_sn_;
    std::atomic<bool> running_ { false };
    std::atomic<bool> stream_faulted_ { false };
    YAML::Node config_;
    std::string trigger_source_;
    int64_t trigger_activation_;
    PixelFormat target_format_ = PixelFormat::BGR;
    bool use_cuda_cvt_ = false;
    // MV_CC_GetImageBuffer 的缓存节点数。SDK 默认只给 1 个, 而 read() 要占着这个节点
    // 做完整张 Bayer->BGR 才归还, 于是一帧处理超过帧周期就立刻 0x4002 溢出并丢帧。
    unsigned int image_node_num_ = 8;
    // 丢帧统计, 只由采集线程访问。
    unsigned int last_frame_num_ = 0;
    bool has_last_frame_num_ = false;
    uint64_t dropped_frames_ = 0;
    // 由 SDK 的回调线程累加, 采集线程读。
    std::atomic<uint64_t> overflow_events_ { 0 };
};
} // namespace dts_vision::tools::io
