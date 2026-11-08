#include "tools/io/camera_factory.hpp"

#include "tools/logger.hpp"
#include "tools/utils.hpp"

#include <stdexcept>

#ifdef USE_HikSDK
    #include "tools/io/hikrobot/hikrobot.hpp"
#endif
#ifdef USE_MvSDK
    #include "tools/io/mindvision/mindvision.hpp"
#endif
#include "tools/io/usbcamera/usbcamera.hpp"
#include "tools/io/video_player.hpp"

namespace dts_vision::tools::io {

namespace {

    std::string camera_type_from_config(const YAML::Node& config, const std::string& default_type) {
        return tools::to_upper(config["type"].as<std::string>(default_type));
    }

    [[noreturn]] void throw_unsupported_camera(const std::string& type) {
        throw std::runtime_error("unsupported camera type: " + type);
    }

} // namespace

std::unique_ptr<Camera>
create_camera(const YAML::Node& config, Scheduler& scheduler, const std::string& default_type) {
    const auto type = camera_type_from_config(config, default_type);

    if (type == "VIDEO" || type == "VIDEO_PLAYER") {
        return std::make_unique<VideoPlayer>(config["video"], scheduler);
    }
    if (type == "UVC") {
        return std::make_unique<UVCCamera>(config["uvc_camera"], &scheduler);
    }
    if (type == "HIK" || type == "HIK_CAMERA") {
#ifdef USE_HikSDK
        return std::make_unique<HikCamera>(config["hik_camera"], scheduler);
#else
        DTS_ERROR("Hik camera requested but USE_HikSDK is disabled");
        throw_unsupported_camera(type);
#endif
    }
    if (type == "MV" || type == "MV_CAMERA") {
#ifdef USE_MvSDK
        return std::make_unique<MvCamera>(config["mv_camera"], scheduler);
#else
        DTS_ERROR("Mv camera requested but USE_MvSDK is disabled");
        throw_unsupported_camera(type);
#endif
    }

    throw_unsupported_camera(type);
}

std::unique_ptr<Camera> create_camera(const YAML::Node& config, const std::string& default_type) {
    const auto type = camera_type_from_config(config, default_type);

    if (type == "UVC") {
        return std::make_unique<UVCCamera>(config["uvc_camera"]);
    }

    throw_unsupported_camera(type);
}

} // namespace dts_vision::io
