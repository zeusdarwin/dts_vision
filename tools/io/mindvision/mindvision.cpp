#ifdef USE_MvSDK
    #include "tools/io/mindvision/mindvision.hpp"
namespace dts_vision::tools::io {
void MvCamera::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    // 先把采集线程收掉再动句柄: 这个线程可能正卡在 CameraGetImageBuffer 里。
    stop_watchdog();
    stop_source_thread();
    teardown_handle();

    DTS_INFO("mv_camera has stop ");
}

void MvCamera::teardown_handle() {
    if (h_camera_ == 0) {
        return;
    }
    CameraUnInit(h_camera_);
    h_camera_ = 0;
}

bool MvCamera::start_capture() {
    if (h_camera_ == 0) {
        DTS_ERROR("mv_camera: camera is not initialized");
        return false;
    }
    const auto status = CameraPlay(h_camera_);
    if (status != CAMERA_STATUS_SUCCESS) {
        DTS_ERROR("mv_camera: CameraPlay failed: {}", status);
        return false;
    }
    running_ = true;
    return true;
}

ImageFrame MvCamera::read() {
    ImageFrame frame;
    if (h_camera_ == 0) {
        DTS_ERROR("mv_camera: camera is not initialized");
        return frame;
    }
    tSdkFrameHead head {};
    BYTE* raw = nullptr;
    auto status = CameraGetImageBuffer(h_camera_, &head, &raw, 100);
    if (status != CAMERA_STATUS_SUCCESS) {
        DTS_ERROR("Failed to get image buffer!");
        return frame;
    }

    const auto current_time = Clock::now();
    const auto half_exposure = std::chrono::microseconds(static_cast<long>(get_ExposureTime() / 2));
    decode_frame(head, raw, frame.src_img);
    CameraReleaseImageBuffer(h_camera_, raw);

    frame.timestamp = current_time - half_exposure;
    frame.format = PixelFormat::BGR;
    return frame;
}

// 由采集线程调用。以前这里只 CameraPlay 一次, 失败就当成功返回, 相机会永久停在 0 帧。
void MvCamera::restart() {
    DTS_WARN("Restarting camera");
    teardown_handle();

    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!initialize_camera()) {
            // running_ 被置假, 正在退出。
            return;
        }
        apply_config(config_);
        if (start_capture()) {
            DTS_INFO("Camera restarted successfully!");
            return;
        }
        DTS_ERROR("mv_camera: CameraPlay failed after restart, retrying");
        teardown_handle();
    }
}
void MvCamera::init() {
    load(config_);
}
void MvCamera::load(const YAML::Node& config) {
    if (!initialize_camera()) {
        DTS_ERROR("Failed to initialize camera");
        return;
    }
    apply_config(config);
}

void MvCamera::apply_config(const YAML::Node& config) {
    CameraGetCapability(h_camera_, &t_capability_);
    CameraSetAeState(h_camera_, false);
    // Match the legacy MindVision setup: let the SDK deliver BGR directly.
    // This avoids RGB/BGR channel-order ambiguity in downstream OpenCV code.
    const auto format_status = CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_BGR8);
    if (format_status != CAMERA_STATUS_SUCCESS) {
        DTS_WARN("MvCamera: failed to set ISP output format to BGR8: {}", format_status);
    }
    CameraSetTriggerMode(h_camera_, 0);
    int frame_speed = config["frame_speed"].as<int>();
    CameraSetFrameSpeed(h_camera_, frame_speed);
    auto exposure_time = config["exposure_time"].as<double>();
    // 初始化阶段采集线程还没起来, 直接下发。
    write_exposure_time(exposure_time);
    setup_white_balance(config);
    if (config["analog_gain"]) {
        auto analog_gain = config["analog_gain"].as<double>();
        set_analog_gain(analog_gain);
    }
    if (config["gamma"]) {
        auto gamma = config["gamma"].as<double>();
        set_gamma(gamma);
    }
    #ifdef USE_TRT
    use_cuda_cvt_ = config["use_cuda_cvt"].as<bool>();
    #endif
    configure_watchdog(config);
}

void MvCamera::decode_frame(const tSdkFrameHead& head, BYTE* raw, cv::Mat& target) {
    // MindVision's ISP performs the correct Bayer pattern, white balance and
    // channel ordering.  CameraGetImageBuffer returns the sensor buffer;
    // demosaicing it ourselves can swap the Bayer pattern and produce a green
    // image.  The ISP output was configured as BGR8 in load().
    target.create(head.iHeight, head.iWidth, CV_8UC3);
    const auto status = CameraImageProcess(h_camera_, raw, target.data,
                                           const_cast<tSdkFrameHead*>(&head));
    if (status != CAMERA_STATUS_SUCCESS) {
        DTS_ERROR("MvCamera: CameraImageProcess failed: {}", status);
        target.release();
    }
}
void MvCamera::write_exposure_time(double exposure_time) {
    if (h_camera_ == 0) {
        return;
    }
    double exposure_line_time = 0.0;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    exposure_time = std::clamp(
        exposure_time,
        t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time,
        t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time
    );
    const auto status = CameraSetExposureTime(h_camera_, exposure_time);
    if (status != CAMERA_STATUS_SUCCESS) {
        DTS_ERROR("MvCamera: CameraSetExposureTime failed: {}", status);
        return;
    }
    // 缓存实际下发的值, get_ExposureTime 直接读它。
    exposure_time_val_ = exposure_time;
    DTS_INFO(
        "MvCamera: set exposure_time {}, min {} max {}",
        exposure_time,
        t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time,
        t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time
    );
}
// 白平衡。CameraSetGain 下发的是数字增益, 100 = 1.0x, 所以三个通道都填 100 等于把
// 白平衡钉死在 1.0/1.0/1.0。RGGB 的绿像素数量是红蓝的两倍、响应也更高, 不做白平衡
// 画面就会明显偏绿。能力位决定能用哪条路: 支持自动就交给 SDK, 不支持就保留相机
// 自己存的那组增益(Camera/Data/*.mvdat), 只有显式给了增益才覆盖。
void MvCamera::setup_white_balance(const YAML::Node& config) {
    const auto& isp = t_capability_.sIspCapacity;
    if (isp.bMonoSensor) {
        DTS_INFO("MvCamera: mono sensor, skip white balance");
        return;
    }
    DTS_INFO(
        "MvCamera: white balance capability: auto {}, once {}",
        isp.bAutoWb ? 1 : 0,
        isp.bWbOnce ? 1 : 0
    );

    const bool auto_wb = config["auto_wb"] ? config["auto_wb"].as<bool>() : true;
    const bool wb_once = config["wb_once"] ? config["wb_once"].as<bool>() : false;
    const bool has_gain = config["r_gain"] && config["b_gain"] && config["g_gain"];

    if (auto_wb && isp.bAutoWb) {
        set_auto_wb(true);
        log_rgb_gain();
        return;
    }
    if (auto_wb) {
        DTS_WARN("MvCamera: auto white balance unsupported, keep the gains stored in the camera");
    } else if (isp.bAutoWb) {
        // 自动模式会持续覆盖下面下发的增益, 手动模式必须先把它关掉。
        set_auto_wb(false);
    }

    if (has_gain) {
        auto r_gain = config["r_gain"].as<double>();
        auto b_gain = config["b_gain"].as<double>();
        auto g_gain = config["g_gain"].as<double>();
        set_rgb_gain(r_gain, b_gain, g_gain);
    } else if (wb_once && isp.bWbOnce) {
        once_wb();
    }
    log_rgb_gain();
}
void MvCamera::log_rgb_gain() const {
    int r_gain = 0, g_gain = 0, b_gain = 0;
    const auto status = CameraGetGain(h_camera_, &r_gain, &g_gain, &b_gain);
    if (status != CAMERA_STATUS_SUCCESS) {
        DTS_WARN("MvCamera: CameraGetGain failed: {}", status);
        return;
    }
    DTS_INFO(
        "MvCamera: effective rgb gain = {}, {}, {} (100 = 1.0x)",
        r_gain,
        g_gain,
        b_gain
    );
}
void MvCamera::set_auto_wb(bool enable) {
    const auto status = CameraSetWbMode(h_camera_, enable ? TRUE : FALSE);
    if (status != CAMERA_STATUS_SUCCESS) {
        DTS_WARN(
            "MvCamera: failed to set white balance mode to {}: {}",
            enable ? "auto" : "manual",
            status
        );
        return;
    }
    DTS_INFO("MvCamera: white balance mode = {}", enable ? "auto" : "manual");
}
void MvCamera::once_wb() {
    const auto status = CameraSetOnceWB(h_camera_);
    if (status != CAMERA_STATUS_SUCCESS) {
        DTS_WARN("MvCamera: CameraSetOnceWB failed: {}", status);
        return;
    }
    DTS_INFO("MvCamera: triggered one-shot white balance");
}
void MvCamera::set_rgb_gain(int r_gain, int b_gain, int g_gain) {
    r_gain = std::clamp(
        r_gain,
        t_capability_.sRgbGainRange.iRGainMin,
        t_capability_.sRgbGainRange.iRGainMax
    );
    b_gain = std::clamp(
        b_gain,
        t_capability_.sRgbGainRange.iBGainMin,
        t_capability_.sRgbGainRange.iBGainMax
    );
    g_gain = std::clamp(
        g_gain,
        t_capability_.sRgbGainRange.iGGainMin,
        t_capability_.sRgbGainRange.iGGainMax
    );
    CameraSetGain(h_camera_, r_gain, g_gain, b_gain);
    DTS_INFO(
        "MvCamera: set rgb_gain = {}, {}, {}, r: min {} max {} , g: min {} max {} , b: min {} max {}",
        r_gain,
        g_gain,
        b_gain,
        t_capability_.sRgbGainRange.iRGainMin,
        t_capability_.sRgbGainRange.iRGainMax,
        t_capability_.sRgbGainRange.iGGainMin,
        t_capability_.sRgbGainRange.iGGainMax,
        t_capability_.sRgbGainRange.iBGainMin,
        t_capability_.sRgbGainRange.iBGainMax
    );
}
void MvCamera::set_analog_gain(double analog_gain) {
    analog_gain = std::clamp(
        analog_gain,
        (double)t_capability_.sExposeDesc.uiAnalogGainMin,
        (double)t_capability_.sExposeDesc.uiAnalogGainMax
    );
    CameraSetAnalogGain(h_camera_, analog_gain);
    DTS_INFO(
        "MvCamera: set analog_gain {}, min {} max {}",
        analog_gain,
        t_capability_.sExposeDesc.uiAnalogGainMin,
        t_capability_.sExposeDesc.uiAnalogGainMax
    );
}
void MvCamera::set_gamma(double gamma) {
    gamma = std::clamp(
        gamma,
        (double)t_capability_.sGammaRange.iMin,
        (double)t_capability_.sGammaRange.iMax
    );
    CameraSetGamma(h_camera_, gamma);
    DTS_INFO(
        "MvCamera: set gamma {}, min {} max {}",
        gamma,
        t_capability_.sGammaRange.iMin,
        t_capability_.sGammaRange.iMax
    );
}
bool MvCamera::initialize_camera() {
    teardown_handle();
    while (running_) {
        int i_camera_counts = 1;
        int i_status = -1;
        tSdkCameraDevInfo t_camera_enum_list;
        i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
        DTS_INFO("Enumerate state = {}", i_status);
        DTS_INFO("Found camera count = {}", i_camera_counts);

        // 没有连接设备
        if (i_camera_counts == 0) {
            DTS_ERROR("No camera found!");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

        // 初始化失败
        DTS_INFO("MvCamera: Init state = {}", i_status);
        if (i_status != CAMERA_STATUS_SUCCESS) {
            DTS_ERROR("MvCamera: Init failed!");
            h_camera_ = 0;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        DTS_INFO("Camera initialized successfully");
        return true;
    }

    return false;
}
} // namespace dts_vision::tools::io
#endif
