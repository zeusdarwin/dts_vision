#pragma once
#include "tools/common/image.hpp"
#include "tools/io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/scheduler/scheduler.hpp"
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <yaml-cpp/node/node.h>
namespace dts_vision::tools::io {
class VideoPlayer: public Camera {
public:
    VideoPlayer(const YAML::Node& config, Scheduler& scheduler): Camera(&scheduler) {
        path_ = config["path"].as<std::string>();
        fps_ = config["fps"].as<int>();
        loop_ = config["loop"].as<bool>();
        start_frame_ = config["start_frame"].as<int>();
    }
    ~VideoPlayer() {
        stop();
    }
    bool start_capture() override {
        cap_.open(path_);
        if (!cap_.isOpened()) {
            DTS_ERROR("open {} failed", path_);
            return false;
        }
        cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
        next_frame_time_ = Clock::now();
        running_ = true;
        return true;
    }
    bool is_running() const override {
        return running_;
    }
    ImageFrame read() override {
        const auto frame_interval =
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / fps_));
        std::this_thread::sleep_until(next_frame_time_);
        next_frame_time_ += frame_interval;

        cv::Mat frame_bgr;
        cap_ >> frame_bgr;

        if (frame_bgr.empty()) {
            if (loop_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
            } else {
                running_ = false;
            }
            return {};
        }

        ImageFrame frame;
        frame.src_img = std::move(frame_bgr);
        frame.timestamp = Clock::now();
        frame.format = PixelFormat::BGR;
        return frame;
    }
    void stop() override {
        running_ = false;
        stop_watchdog();
        stop_source_thread();
        cap_.release();
        DTS_INFO("Video closed successfully: {}", path_);
    }
    std::string path_;
    int fps_;
    bool loop_;
    int start_frame_;
    std::atomic<bool> running_ { false };
    cv::VideoCapture cap_;
    TimePoint next_frame_time_ {};
};
} // namespace dts_vision::tools::io
