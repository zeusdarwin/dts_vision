#pragma once
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/common.hpp"
#include "tools/common/impl.hpp"
#include <opencv2/core/types.hpp>
#include <yaml-cpp/node/node.h>
namespace dts_vision::auto_buff {
class RuneDetector {
public:
    RuneDetector(const YAML::Node& config);
    RuneDetection detect(const CommonFrame& frame, const cv::Rect& focus, EnemyColor enemy_color);
    DTS_IMPL_DEFINITION(RuneDetector)
};
} // namespace dts_vision::auto_buff
