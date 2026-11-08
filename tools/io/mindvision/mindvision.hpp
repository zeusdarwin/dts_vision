
#pragma once
#include "CameraApi.h"
#include "tools/common/image.hpp"
#include "tools/io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/scheduler/scheduler.hpp"
#include <algorithm>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#ifdef USE_TRT
    #include "tools/cuda/cvtcolor.hpp"
#endif
namespace dts_vision::tools::io {
class MvCamera: public Camera {
public:
    MvCamera(const YAML::Node& config, Scheduler& scheduler): Camera(&scheduler) {
        config_ = config;
        running_ = true;
        CameraSdkInit(0);
    }
    ~MvCamera() {
        stop();
    }
    void stop() override;
    void restart() override;
    bool is_running() const override {
        return running_;
    }
    ImageFrame read() override;
    bool start_capture() override;
    void init() override;
    void load(const YAML::Node& config);
    void apply_config(const YAML::Node& config);
    // 缓存值, 不要再去问 SDK: CameraGetExposureTime 是控制通道调用, 而这个 getter 会被
    // scheduler 的 auto_exposure task 在工作线程上调用。
    double get_ExposureTime() const override {
        return exposure_time_val_;
    }
    // 只会被采集线程调用, 见 Camera::flush_exposure_request 上面的注释。
    void write_exposure_time(double exposure_time) override;
    void setup_white_balance(const YAML::Node& config);
    void log_rgb_gain() const;
    void set_auto_wb(bool enable);
    void once_wb();
    void set_rgb_gain(int r_gain, int b_gain, int g_gain);
    void set_analog_gain(double analog_gain);
    void set_gamma(double gamma);
    bool initialize_camera();
    // CameraUnInit, 幂等。
    void teardown_handle();
    void decode_frame(const tSdkFrameHead& head, BYTE* raw, cv::Mat& target);
    std::atomic<bool> running_ { false };
    YAML::Node config_;
    CameraHandle h_camera_ = 0;
    tSdkCameraCapbility t_capability_; // 设备描述信息
    double exposure_time_val_ {};
    bool use_cuda_cvt_ = false;
};
} // namespace dts_vision::io
