#pragma once
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/common.hpp"
#include "tools/common/impl.hpp"
#include <yaml-cpp/node/node.h>
namespace dts_vision::auto_aim {
class ArmorTracker {
public:
    ArmorTracker(const YAML::Node& config);
    [[nodiscard]] ArmorTarget track(
        ArmorDetection& armor_detectioj,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world,
        int frame_id
    );
    int get_count();
    void reset_count();
    void set_sentry(bool is_sentry);
    DTS_IMPL_DEFINITION(ArmorTracker)
};
} // namespace dts_vision::auto_aim
