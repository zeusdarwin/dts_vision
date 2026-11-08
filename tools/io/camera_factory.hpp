#pragma once

#include "tools/io/camera.hpp"
#include <memory>
#include <string>
#include <yaml-cpp/node/node.h>

namespace dts_vision::tools::io {

std::unique_ptr<Camera> create_camera(
    const YAML::Node& config,
    Scheduler& scheduler,
    const std::string& default_type = "hik"
);

std::unique_ptr<Camera>
create_camera(const YAML::Node& config, const std::string& default_type = "uvc");

} // namespace dts_vision::tools::io
