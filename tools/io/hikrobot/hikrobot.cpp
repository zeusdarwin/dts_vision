#ifdef USE_HikSDK
    #include "tools/io/hikrobot/hikrobot.hpp"
    #ifdef USE_TRT
        #include "tools/cuda/cvtcolor.hpp"
    #endif
namespace dts_vision::tools::io {
void HikCamera::load(const YAML::Node& config) {
    std::string target_sn = config["target_sn"].as<std::string>();
    if (!initialize_camera(target_sn)) {
        DTS_ERROR("Failed to initialize camera with SN: {}", target_sn);
        return;
    }
    apply_config(config);
}

void HikCamera::apply_config(const YAML::Node& config) {
    // 取流缓存节点数, 由 start_capture 在 StartGrabbing 之前下发。不配就用默认值。
    if (config["image_node_num"]) {
        image_node_num_ = static_cast<unsigned int>(std::max(1, config["image_node_num"].as<int>()));
    }
    auto acquisition_frame_rate = config["acquisition_frame_rate"].as<double>();
    set_AcquisitionFrameRate(acquisition_frame_rate);
    auto exposure_time = config["exposure_time"].as<double>();
    // 初始化阶段采集线程还没起来, 直接下发。
    write_exposure_time(exposure_time);
    auto gain = config["gain"].as<double>();
    set_Gain(gain);
    auto gamma = config["gamma"].as<double>();
    set_Gamma(gamma);
    auto adc_bit_depth = config["adc_bit_depth"].as<std::string>();
    set_ADCBitDepth(adc_bit_depth);
    auto pixel_format = config["pixel_format"].as<std::string>();
    set_PixelFormat(pixel_format);
    auto acquisition_frame_rate_enable = config["acquisition_frame_rate_enable"].as<bool>();
    set_AcquisitionFrameRateEnable(acquisition_frame_rate_enable);
    auto width = config["width"].as<int>();
    set_Width(width);
    auto height = config["height"].as<int>();
    set_Height(height);
    auto offset_x = config["offset_x"].as<int>();
    set_OffsetX(offset_x);
    auto offset_y = config["offset_y"].as<int>();
    set_OffsetY(offset_y);
    auto reverse_x = config["reverse_x"].as<bool>();
    set_ReverseX(reverse_x);
    auto reverse_y = config["reverse_y"].as<bool>();
    set_ReverseY(reverse_y);
    auto trigger_mode = config["trigger_mode"].as<std::string>();
    set_TriggerMode(trigger_mode);
    auto trigger_source = config["trigger_source"].as<std::string>();
    set_TriggerSource(trigger_source);
    auto trigger_activation = config["trigger_activation"].as<std::string>();
    set_TriggerActivation(trigger_activation);

    auto format_str = config["format"].as<std::string>();
    target_format_ = string2PixelFormat(format_str);
    #ifdef USE_TRT
    use_cuda_cvt_ = config["use_cuda_cvt"].as<bool>();
    #endif
    configure_watchdog(config);
    DTS_INFO("Camera parameters set successfully!");
}
void HikCamera::teardown_handle() {
    if (!camera_handle_) {
        return;
    }
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    int n_ret = MV_CC_DestroyHandle(camera_handle_);
    if (n_ret != MV_OK) {
        DTS_WARN("MV_CC_DestroyHandle failed: {}", n_ret);
    }
    camera_handle_ = nullptr;
}

void HikCamera::on_stream_exception(MV_CC_STREAM_EXCEPTION_INFO* info, void* user) {
    auto* self = static_cast<HikCamera*>(user);
    if (!info || !self) {
        return;
    }
    switch (info->enExceptionType) {
        // 取流被 SDK 中止, 不重启就再也不会出帧了。
        case MV_CC_STREAM_EXCEPTION_DISCONNECTED:
        case MV_CC_STREAM_EXCEPTION_DEVICE:
            DTS_ERROR("hik_camera: stream aborted by SDK, type=0x{:x}", (int)info->enExceptionType);
            self->stream_faulted_.store(true, std::memory_order_release);
            break;
        // 缓存列表满 = 取图不及时或者带宽不够, 只掉帧, 不用重启。溢出一旦开始就是
        // 持续的(实测 Jetson 上 1471 秒报了 1777 次), 每次都打一行日志等于在回调线程
        // 上以 1Hz+ 的频率做 IO, 反过来又加重取图不及时。这里只计数, 由采集线程
        // 在 account_dropped_frames 里定期汇总。
        case MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW:
            self->overflow_events_.fetch_add(1, std::memory_order_relaxed);
            break;
        case MV_CC_STREAM_EXCEPTION_RECONNECTION:
            DTS_WARN("hik_camera: stream exception 0x{:x}", (int)info->enExceptionType);
            break;
        default:
            break;
    }
}

// 只由采集线程调用。nFrameNum 是相机侧的帧号, 每出一帧 +1, 所以它和我们实际取到的
// 帧数之差就是被丢掉的帧数。这件事下游完全看不见: 时间戳取在出队时刻, 丢帧只会让
// 相邻两帧的 dt 变大, 看起来和"目标那一瞬间动得快"无法区分, 而 ArmorTarget 的
// process_noise 里位置项是 0.25*dt^4、速度项是 dt^2, dt 一错速度和角速度就跟着错。
void HikCamera::account_dropped_frames(const MV_FRAME_OUT_INFO_EX& info) {
    if (has_last_frame_num_) {
        // 帧号是 32 位无符号, 溢出回绕靠无符号减法自然处理。
        const unsigned int advance = info.nFrameNum - last_frame_num_;
        if (advance > 1) {
            dropped_frames_ += advance - 1;
        }
    }
    last_frame_num_ = info.nFrameNum;
    has_last_frame_num_ = true;

    // 定期汇总, 不要每帧打日志。
    using namespace std::chrono;
    static auto last_report = steady_clock::now();
    const auto now = steady_clock::now();
    if (now - last_report < seconds(10)) {
        return;
    }
    const auto overflows = overflow_events_.exchange(0, std::memory_order_relaxed);
    const auto dropped = dropped_frames_;
    const double window_s = duration<double>(now - last_report).count();
    dropped_frames_ = 0;
    last_report = now;
    if (dropped == 0 && overflows == 0) {
        return;
    }
    DTS_WARN(
        "hik_camera: dropped {} frames ({:.1f}/s) over the last {:.0f}s, buffer overflow x{}; "
        "取图跟不上出图, dt 会抖, 考虑降低 acquisition_frame_rate",
        dropped,
        dropped / window_s,
        window_s,
        overflows
    );
}

void HikCamera::on_device_exception(unsigned int msg_type, void* user) {
    auto* self = static_cast<HikCamera*>(user);
    if (!self) {
        return;
    }
    if (msg_type == MV_EXCEPTION_DEV_DISCONNECT) {
        DTS_ERROR("hik_camera: device disconnected");
        self->stream_faulted_.store(true, std::memory_order_release);
    } else {
        DTS_WARN("hik_camera: device exception 0x{:x}", msg_type);
    }
}

void HikCamera::register_exception_callbacks() {
    int n_ret = MV_CC_RegisterStreamExceptionCallBack(camera_handle_, on_stream_exception, this);
    if (n_ret != MV_OK) {
        DTS_WARN("MV_CC_RegisterStreamExceptionCallBack failed: {}", n_ret);
    }
    n_ret = MV_CC_RegisterExceptionCallBack(camera_handle_, on_device_exception, this);
    if (n_ret != MV_OK) {
        DTS_WARN("MV_CC_RegisterExceptionCallBack failed: {}", n_ret);
    }
}
void HikCamera::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    // 先把采集线程收掉再动句柄: 这个线程可能正卡在 MV_CC_GetImageBuffer 里。
    stop_watchdog();
    stop_source_thread();
    teardown_handle();
    DTS_INFO("hik_camera has stop ");
}

bool HikCamera::start_capture() {
    if (!camera_handle_) {
        DTS_ERROR("hik_camera: camera is not initialized");
        return false;
    }
    // 必须在 StartGrabbing 之前下发。MvCameraControl.h 里写明 MV_CC_GetImageBuffer
    // 在不 Free 的情况下最多能输出的节点数就等于这里设置的值, 而默认是 1 个。read()
    // 要占着节点做完整张 Bayer->BGR 才 Free, 所以默认配置下一帧处理只要超过帧周期
    // (250fps 时是 4ms), 相机就没地方放下一帧, 立刻触发 0x4002 并静默丢帧。
    int n_ret = MV_CC_SetImageNodeNum(camera_handle_, image_node_num_);
    if (n_ret != MV_OK) {
        DTS_WARN("hik_camera: MV_CC_SetImageNodeNum({}) failed: {}", image_node_num_, n_ret);
    } else {
        DTS_INFO("hik_camera: image node num set to {}", image_node_num_);
    }

    n_ret = MV_CC_StartGrabbing(camera_handle_);
    if (n_ret != MV_OK) {
        DTS_ERROR("Failed to start camera grabbing: {}", n_ret);
        return false;
    }
    stream_faulted_.store(false, std::memory_order_release);
    has_last_frame_num_ = false;
    running_ = true;
    return true;
}

ImageFrame HikCamera::read() {
    ImageFrame img_frame;
    if (!camera_handle_) {
        DTS_ERROR("hik_camera: camera is not initialized");
        return img_frame;
    }

    Frame frame {};
    int n_ret = MV_CC_GetImageBuffer(camera_handle_, &frame.out_frame, 100);
    if (n_ret != MV_OK) {
        DTS_ERROR("MV_CC_GetImageBuffer fail: {}", n_ret);
        return img_frame;
    }

    const auto current_time = Clock::now();
    const auto& info = frame.out_frame.stFrameInfo;
    account_dropped_frames(info);
    // 用这一帧自己的曝光值, 而不是缓存的请求值: 后者可能比实际下发晚一个控制周期。
    const double exposure_us =
        info.fExposureTime > 0.0f ? (double)info.fExposureTime : get_ExposureTime();
    const auto half_exposure = std::chrono::microseconds(static_cast<long>(exposure_us / 2));
    frame.timestamp = current_time - half_exposure;
    return to_image_frame(frame);
}

// 由采集线程调用。以前这里只试一次 StartGrabbing, 失败就直接 return, 于是相机
// 永久停在 0 帧(实测 19:49 那次 "Failed to start grabbing after restart" 之后再也没出帧)。
void HikCamera::restart() {
    DTS_WARN("Restarting camera");
    stream_faulted_.store(false, std::memory_order_release);
    teardown_handle();

    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!initialize_camera(target_sn_)) {
            // running_ 被置假, 正在退出。
            return;
        }
        apply_config(config_);
        if (start_capture()) {
            DTS_INFO("Camera restarted successfully!");
            return;
        }
        DTS_ERROR("hik_camera: start grabbing failed after restart, retrying");
        teardown_handle();
    }
}
ImageFrame HikCamera::to_image_frame(Frame& f) {
    ImageFrame img_frame {
        .format = target_format_,
        .timestamp = f.timestamp,
    };
    const auto& info = f.out_frame.stFrameInfo;
    cv::Mat src(cv::Size(info.nWidth, info.nHeight), CV_8U, f.out_frame.pBufAddr);
    const auto pixel_type = info.enPixelType;
    const auto& ref_cvt = get_cvt_map();
    auto it = ref_cvt.find(pixel_type);
    if (it == ref_cvt.end()) {
        DTS_ERROR("hik_camera: unsupported pixel type 0x{:x}", (int)pixel_type);
        MV_CC_FreeImageBuffer(camera_handle_, &f.out_frame);
        return img_frame;
    }
    int cvt_code = it->second;
    if (cvt_code != -1) {
    #ifdef USE_TRT
        if (use_cuda_cvt_) {
            static tools::__cuda::CvtColor cvt;
            cvt.process(src, img_frame.src_img, cvt_code);
        } else {
    #endif
            cv::cvtColor(src, img_frame.src_img, cvt_code);
    #ifdef USE_TRT
        }
    #endif

    } else {
        img_frame.src_img = src.clone();
    }
    MV_CC_FreeImageBuffer(camera_handle_, &f.out_frame);
    return img_frame;
}

bool HikCamera::initialize_camera(const std::string& target_sn) {
    target_sn_ = target_sn;
    teardown_handle();

    while (running_) {
        MV_CC_DEVICE_INFO_LIST device_list = { 0 };
        int n_ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        if (n_ret != MV_OK) {
            DTS_ERROR("MV_CC_EnumDevices failed, error code: {}", n_ret);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        if (device_list.nDeviceNum == 0) {
            DTS_ERROR("No USB cameras found");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        DTS_INFO("Found {} USB camera(s):", device_list.nDeviceNum);
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            auto info = device_list.pDeviceInfo[i];
            const char* sn =
                reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
            DTS_INFO("[ {} ] SN = {}", i, sn);
        }

        int sel = -1;
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            auto info = device_list.pDeviceInfo[i];
            const char* sn =
                reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
            if (target_sn == sn) {
                sel = i;
                break;
            }
        }

        if (sel < 0) {
            DTS_ERROR("Camera with serial {} not found", target_sn);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        DTS_INFO("Selecting camera at index {} (SN={})", sel, target_sn);

        n_ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[sel]);
        if (n_ret != MV_OK) {
            DTS_ERROR("MV_CC_CreateHandle failed: {}", n_ret);
            camera_handle_ = nullptr;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        n_ret = MV_CC_OpenDevice(camera_handle_);
        if (n_ret != MV_OK) {
            DTS_ERROR("MV_CC_OpenDevice failed: {}", n_ret);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        register_exception_callbacks();
        DTS_INFO("Camera initialized successfully");
        return true;
    }
    return false;
}
} // namespace dts_vision::io
#endif
