#pragma once
#include "tools/common/image.hpp"
#include "tools/io/camera.hpp"
#include "tools/logger.hpp"
#include <stdexcept>
#include <thread>
#include <yaml-cpp/node/node.h>
namespace dts_vision::tools::io {
class UVCCamera: public Camera {
public:
    explicit UVCCamera(const YAML::Node& config, Scheduler* scheduler = nullptr):
        Camera(scheduler) {
        config_ = config;
        DTS_INFO("try get device_name by ls -l /dev/v4l/by-id/  ls -l /dev/v4l/by-path/");
        try {
            device_name_ = config["device_name"].as<std::string>();

            fps_ = config["fps"].as<int>();

            width_ = config["width"].as<int>();

            height_ = config["height"].as<int>();

            exposure_ = config["exposure"].as<double>();

            gain_ = config["gain"].as<double>();

            gamma_ = config["gamma"].as<double>();

        } catch (const std::exception& e) {
            DTS_ERROR("Failed to load config: {}", e.what());
            throw std::runtime_error("Failed to load UVC config");
        }
        configure_watchdog(config);
    }
    ~UVCCamera() {
        stop();
    }
    void stop() override {
        if (!running_)
            return;

        running_ = false;
        stop_watchdog();
        stop_source_thread();

        cap_.release();

        DTS_INFO("uvc: {} stopped", device_name_);
    }
    bool set_and_check(
        cv::VideoCapture& cap,
        int prop,
        double value,
        const std::string& name,
        double tol = 1e-3
    ) {
        if (!cap.set(prop, value)) {
            DTS_WARN("{} set() failed", name);
            return false;
        }

        double actual = cap.get(prop);

        if (std::abs(actual - value) > tol) {
            DTS_WARN("{} mismatch. requested={} actual={}", name, value, actual);
            return false;
        }

        DTS_INFO("{} OK = {}", name, actual);
        return true;
    }
    void start() {
        if (running_) {
            DTS_WARN("Camera already running.");
            return;
        }

        DTS_INFO("Starting camera: {}", device_name_);
        constexpr int max_attempts = 5;
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            cap_.release();
            if (!cap_.open(device_name_, cv::CAP_V4L2)) {
                DTS_WARN("Failed to open camera (attempt {}/{})", attempt, max_attempts);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                continue;
            }

            set_and_check(cap_, cv::CAP_PROP_FOURCC,
                          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), "FOURCC");
            // V4L2 applies the size after both dimensions have been set.
            // Do not inspect width or short-circuit before setting height.
            const bool width_set = cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            const bool height_set = cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
            const bool fps_set = cap_.set(cv::CAP_PROP_FPS, fps_);
            const double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
            const double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
            const double actual_fps = cap_.get(cv::CAP_PROP_FPS);
            const bool format_ok = width_set && height_set && fps_set
                && std::abs(actual_width - width_) < 1
                && std::abs(actual_height - height_) < 1
                && std::abs(actual_fps - fps_) < 1e-3;
            DTS_INFO("uvc: {} requested={}x{}@{} actual={}x{}@{}",
                     device_name_, width_, height_, fps_, actual_width, actual_height, actual_fps);
            if (format_ok && cap_.isOpened())
                break;

            DTS_WARN("Camera format mismatch (attempt {}/{}), reopening", attempt, max_attempts);
            cap_.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        if (!cap_.isOpened()
            || std::abs(cap_.get(cv::CAP_PROP_FRAME_WIDTH) - width_) > 1
            || std::abs(cap_.get(cv::CAP_PROP_FRAME_HEIGHT) - height_) > 1) {
            DTS_ERROR("Camera failed to open at required resolution {}x{}", width_, height_);
            cap_.release();
            return;
        }
        // UVC drivers may reset exposure when format or frame rate changes.
        // Disable auto exposure only after the stream format is fixed.
        set_and_check(cap_, cv::CAP_PROP_AUTO_EXPOSURE, 1, "AUTO_EXPOSURE");
        set_and_check(cap_, cv::CAP_PROP_EXPOSURE, exposure_, "EXPOSURE");
        set_and_check(cap_, cv::CAP_PROP_GAIN, gain_, "GAIN");
        set_and_check(cap_, cv::CAP_PROP_GAMMA, gamma_, "GAMMA");

        // Do not mark the camera as running until the driver delivers a real frame.
        // Some UVC devices accept the format controls but fail to stream briefly.
        cv::Mat warmup;
        bool frame_ok = false;
        for (int i = 0; i < 10; ++i) {
            if (cap_.read(warmup) && !warmup.empty()) {
                frame_ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!frame_ok) {
            DTS_ERROR("Camera opened but failed to deliver a frame: {}", device_name_);
            cap_.release();
            return;
        }
        running_ = true;
    }
    bool start_capture() override {
        start();
        return running_;
    }
    bool is_running() const override {
        return running_;
    }
    ImageFrame read() override {
        ImageFrame f;
        if (!cap_.isOpened()) {
            DTS_WARN("uvc: camera device is not open");
            return f;
        }
        cv::Mat mat;

        if (cap_.read(mat)) {
            f.src_img = std::move(mat);
            f.timestamp = Clock::now();
            f.format = PixelFormat::BGR;
        } else {
            DTS_WARN("uvc: {} read_image failed.", device_name_);
        }
        return f;
    }
    // 由采集线程调用。以前这里只 start() 一次, 失败就把 running_ 留在 false, 采集循环
    // 跟着退出, 相机永久停在 0 帧。source_thread_running_ 是退出信号, 不能用 running_。
    void restart() override {
        DTS_WARN("Restarting camera.");
        running_ = false;
        cap_.release();
        while (source_thread_running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            start();
            if (running_) {
                DTS_INFO("uvc: {} restarted successfully", device_name_);
                return;
            }
            DTS_ERROR("uvc: {} failed to restart, retrying", device_name_);
        }
    }
    double get_ExposureTime() const override {
        return exposure_;
    }
    void write_exposure_time(double exposure_time) override {
        exposure_ = exposure_time;
        if (cap_.isOpened()) {
            set_and_check(cap_, cv::CAP_PROP_EXPOSURE, exposure_, "EXPOSURE");
        }
    }
    std::string device_name_;
    int fps_;
    int width_;
    int height_;
    double exposure_;
    double gain_;
    double gamma_;
    YAML::Node config_;
    bool running_ = false;
    cv::VideoCapture cap_;
    TimePoint last_frame_time_;
};
} // namespace dts_vision::io
