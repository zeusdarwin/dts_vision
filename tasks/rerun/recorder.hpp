#pragma once

#include "tools/common/type_common.hpp"
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <opencv2/core/mat.hpp>
#include <string>

namespace dts_vision {
struct VisionDebugCtx;
namespace rerun {

    // Thread-safe process-wide Rerun output. The existing Web and ROS2 outputs call this
    // in parallel; it is deliberately not a replacement for either public interface.
    class Recorder {
    public:
        static Recorder& instance();
        bool enabled() const noexcept;

        void log_image(const cv::Mat& image, const std::string& entity = "images/debug");
        void log_json(const std::string& root, const nlohmann::json& value);
        void log_vision(const VisionDebugCtx& ctx);
        void log_transform(
            const std::string& parent,
            const std::string& child,
            const ISO3& child_in_parent
        );

    private:
        Recorder();
        ~Recorder();
        Recorder(const Recorder&) = delete;
        Recorder& operator=(const Recorder&) = delete;
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace rerun
} // namespace dts_vision
