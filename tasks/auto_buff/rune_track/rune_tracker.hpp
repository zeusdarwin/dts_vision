#pragma once
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/common.hpp"
#include "tools/common/impl.hpp"
#include <yaml-cpp/node/node.h>
namespace dts_vision::auto_buff {
class RuneTracker {
public:
    RuneTracker(const YAML::Node& config);
    DTS_IMPL_DEFINITION(RuneTracker)
    RuneTarget track(
        RuneDetection& detection,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world,
        int frame_id
    );
    int get_count();
    void reset_count();
};
} // namespace dts_vision::auto_buff
