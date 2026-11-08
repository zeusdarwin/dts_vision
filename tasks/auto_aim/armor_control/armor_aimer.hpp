#pragma once
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/base/common.hpp"
#include "tools/common/impl.hpp"
#include <memory>
#include <yaml-cpp/node/node.h>

namespace dts_vision::auto_aim {
class ArmorAimer {
public:
    ArmorAimer(const YAML::Node& config);
    DTS_IMPL_DEFINITION(ArmorAimer)
    [[nodiscard]] GimbalCmd aim(
        const ArmorTarget& target,
        double bullet_speed,
        const AutoAimFsmMode& fsm_mode,
        const ISO3& T_shoot2gimbal_abs,
        const ISO3& T_gimbal2gimbal_abs
    );
    void reset();
    std::pair<double, double> get_yaw_pitch_offset();
    void set_operator_offset(std::pair<double, double> offset);
};
} // namespace dts_vision::auto_aim
