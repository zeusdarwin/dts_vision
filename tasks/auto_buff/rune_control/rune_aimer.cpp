#include "rune_aimer.hpp"
#include "tasks/auto_buff/rune_track/motion_model.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/dta_utils.hpp"
#include "tools/common/type_common.hpp"
#include "tools/logger.hpp"
#include <chrono>
#include <ostream>
namespace dts_vision::auto_buff {
struct RuneAimer::Impl {
    struct Params {
        double sample_total_time;
        int sample_horizon;
        double prediction_delay;
        double max_yaw_acc;
        double max_pitch_acc;
        double shooting_range_w;
        double shooting_range_h;
        double min_enable_pitch_deg;
        double min_enable_yaw_deg;
        void load(const YAML::Node& config) {
            prediction_delay = config["prediction_delay"].as<double>();
            shooting_range_w = config["shooting_range_w"].as<double>();
            shooting_range_h = config["shooting_range_h"].as<double>();
            max_yaw_acc = config["max_yaw_acc"].as<double>();
            max_pitch_acc = config["max_pitch_acc"].as<double>();
            min_enable_pitch_deg = config["min_enable_pitch_deg"].as<double>();
            min_enable_yaw_deg = config["min_enable_yaw_deg"].as<double>();
            sample_total_time = config["sample_total_time"].as<double>();
            sample_horizon = config["sample_horizon"].as<int>();
        }
    } params_;
    Impl(const YAML::Node& config) {
        params_.load(config);
        ballistic_trajectory = BallisticTrajectory::create(config["ballistic_trajectory"]);
        base_yaw_offset_rad = angles::from_degrees(config["base_yaw_offset"].as<double>());
        base_pitch_offset_rad = angles::from_degrees(config["base_pitch_offset"].as<double>());
    }
    struct HitCtx {
        RuneTarget hit_time_target;
        double fly_time;
    };
    int get_select_id(const RuneTarget& target) const noexcept {
        auto& wc = target.fan_wc;
        if (target.voter.mode != motion_model::Voter::Big) {
            return target.fan_wc.get_min_visable_fan_id();
        }
        static std::array<TimePoint, FAN_NUM> fan_aim_times { Clock::now(),
                                                              Clock::now(),
                                                              Clock::now(),
                                                              Clock::now(),
                                                              Clock::now() };
        enum class FSM {
            no_inited,
        };
        return target.fan_wc.get_min_visable_fan_id();
    }
    [[nodiscard]] ControlPoint get_control_point(
        const RuneTarget& target,
        double bullet_speed,
        const ISO3& T_shoot2gimbal_abs,
        const ISO3& T_gimbal2gimbal_abs,
        int aim_id
    ) const noexcept {
        auto T_fans2world = target.get_target_state().get_fan_target_pose();
        return get_control_point(
            T_fans2world[aim_id],
            T_shoot2gimbal_abs,
            T_gimbal2gimbal_abs,
            bullet_speed,
            aim_id
        );
    }
    [[nodiscard]] ControlPoint get_control_point(
        const ISO3& T_fan2world,
        const ISO3& T_shoot2gimbal_abs,
        const ISO3& T_gimbal2gimbal_abs,
        double bullet_speed,
        int aim_id
    ) const noexcept {
        ControlPoint control_point;

        auto xyz_in_shoot = T_fan2world.translation() - T_shoot2gimbal_abs.translation();// t_fan2shoot
        auto desired_control_pitch = ballistic_trajectory->solve_pitch(xyz_in_shoot, bullet_speed);
        if (!desired_control_pitch) {
            control_point.valid = false;
            DTS_ERROR(
                "rune_aimer: get_control_point: Failed to solve pitch armor_pos: [{}, {}, {}], bullet_speed: {}",
                xyz_in_shoot.x(),
                xyz_in_shoot.y(),
                xyz_in_shoot.z(),
                bullet_speed
            );
            return control_point;
        }
        const auto [yaw_offset, pitch_offset] = get_yaw_pitch_offset();
        const double desired_control_yaw = std::atan2(xyz_in_shoot.y(), xyz_in_shoot.x());
        auto R_shoot2control = tools::euler2matrix(Vec3(
            angles::normalize_angle(desired_control_yaw + yaw_offset),
            desired_control_pitch.value() + pitch_offset,
            0.0), tools::EulerOrder::ZYX);
        auto R_shoot2gimbal = T_gimbal2gimbal_abs.linear().inverse() * T_shoot2gimbal_abs.linear();// R_gimbal_abs2gimbal*R_shoot2gimbal_abs = R_shoot2gimbal
        auto R_gimbal2control = R_shoot2control * R_shoot2gimbal.inverse();//  R_shoot2aimer * R_shoot2gimbal = R_aimer2gimbal
        auto ypr = tools::matrix2euler(R_gimbal2control, tools::EulerOrder::ZYX);
        control_point.valid = true;
        control_point.yaw = ypr[0];
        control_point.pitch = ypr[1];
        control_point.aim_point.pose = T_fan2world;
        control_point.aim_id = aim_id;
        return control_point;
    };

    std::optional<HitCtx> get_hit(
        const RuneTarget& target_ready_to_aim,
        double bullet_speed,
        const ISO3& T_shoot2gimbal_abs,
        int aim_id
    ) const noexcept {
        auto hit_time_target = target_ready_to_aim;
        const int simply_select = get_select_id(target_ready_to_aim);
        const auto T_fans2world = hit_time_target.get_target_state().get_fan_target_pose();
        auto prev_pitch_and_fly_time_opt = ballistic_trajectory->solve_pitch_and_flytime(
            T_fans2world[simply_select].translation() - T_shoot2gimbal_abs.translation(),// t_fan2shoot
            bullet_speed
        );
        if (!prev_pitch_and_fly_time_opt) {
            return std::nullopt;
        }
        auto prev_fly_time = prev_pitch_and_fly_time_opt.value().second;

        for (int iter = 0; iter < 10; ++iter) {
            auto i_target = hit_time_target;
            i_target.set_target_state([&](motion_model::State& state) {
                state.predict(prev_fly_time, i_target.voter);
            });
            const auto iter_T_fans2world = i_target.get_target_state().get_fan_target_pose();
            auto iter_pitch_and_fly_time_opt = ballistic_trajectory->solve_pitch_and_flytime(
                iter_T_fans2world[simply_select].translation() - T_shoot2gimbal_abs.translation(),// t_fan2shoot
                bullet_speed
            );
            if (!iter_pitch_and_fly_time_opt) {
                return std::nullopt;
            }
            if (std::abs(iter_pitch_and_fly_time_opt.value().second - prev_fly_time) < 1e-3) {
                prev_fly_time = iter_pitch_and_fly_time_opt.value().second;
                break;
            }

            prev_fly_time = iter_pitch_and_fly_time_opt.value().second;
        }
        const double predict_time = prev_fly_time + params_.prediction_delay;
        hit_time_target.set_target_state([&](auto& state) {
            state.predict(predict_time, hit_time_target.voter);
        });
        return HitCtx {
            .hit_time_target = hit_time_target,
            .fly_time = prev_fly_time,
        };
    }
    RuneTarget last_target;
    TimePoint last_time;
    LimitTrajectory limit_target_traj;
    ControlPoint limit_target_traj_cp0;
    Trajectory<AimPoint, double> limit_aim_traj;
    double last_fly_time;
    GimbalCmd
    aim(const RuneTarget& _target,
        double bullet_speed,
        const ISO3& T_shoot2gimbal_abs,
        const ISO3& T_gimbal2gimbal_abs) noexcept {
        GimbalCmd cmd;
        cmd.appear = false;
        bool is_same = _target.this_id == last_target.this_id;
        double time_in_traj = 0.0;
        // if (is_same) { //状态完全未更新，使用第一次构建的轨迹基于时间进行推进减少重复计算
        //     time_in_traj = std::chrono::duration<double>(Clock::now() - last_time).count();
        // } else {
        is_same = false;
        last_target = _target;
        last_time = Clock::now();
        // }
        auto target = _target;
        target.set_target_state([&](motion_model::State& state) {
            state.predict(Clock::now(), target.voter);
        });
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };

        const int horizon = make_even(params_.sample_horizon);
        const double dt = params_.sample_total_time / horizon;
        int now_select = get_select_id(target);
        if (!is_same) {
            auto hit_ctx_opt = get_hit(
                target,
                bullet_speed,
                T_shoot2gimbal_abs,
                now_select
            ); //直接算到击中目标时间
            if (!hit_ctx_opt) {
                return cmd;
            }
            auto hit_ctx = hit_ctx_opt.value();
            auto control_point0 = get_control_point(
                hit_ctx.hit_time_target,
                bullet_speed,
                T_shoot2gimbal_abs,
                T_gimbal2gimbal_abs,
                now_select
            );
            if (!control_point0.valid) {
                return cmd;
            }
            last_fly_time = hit_ctx.fly_time;
            auto sample_once = [&](double t,
                                   const RuneTarget& base_target,
                                   const ControlPoint& _control_point0,
                                   GimbalState& out_gs,
                                   AimPoint& out_ap) -> bool {
                auto tmp_target = base_target;
                tmp_target.set_target_state([&](motion_model::State& state) {
                    state.predict(t, tmp_target.voter);
                }); //保证轨迹完全为不同时间对同一状态的瞄准控制点
                int this_select = now_select;
                auto hit_opt = get_hit(tmp_target, bullet_speed, T_shoot2gimbal_abs, this_select);
                if (!hit_opt)
                    return false;

                auto control_point = get_control_point(
                    hit_opt->hit_time_target,
                    bullet_speed,
                    T_shoot2gimbal_abs,
                    T_gimbal2gimbal_abs,
                    this_select
                );
                if (!control_point.valid)
                    return false;

                out_gs.aim_id = control_point.aim_id;
                out_gs.yaw_state.p = angles::normalize_angle(control_point.yaw - _control_point0.yaw);
                out_gs.pitch_state.p = angles::normalize_angle(control_point.pitch - _control_point0.pitch);
                out_ap = control_point.aim_point;

                return true;
            };
            auto push_sample =
                [&](auto& traj_gs, auto& traj_ap, double t, const ControlPoint& _control_point0
                ) -> bool {
                GimbalState gs;
                AimPoint ap;
                if (!sample_once(t, target, _control_point0, gs, ap)) {
                    return false;
                }
                traj_gs.push_back(gs, t);
                traj_ap.push_back(ap, t);
                return true;
            };

            auto build_traj = [&](auto& traj_gs,
                                  auto& traj_ap,
                                  const ControlPoint& _control_point0,

                                  int horizon,
                                  double dt) -> bool {
                int half = horizon / 2;
                for (int i = half; i >= 1; --i) {
                    if (!push_sample(traj_gs, traj_ap, -i * dt, _control_point0)) {
                        return false;
                    }
                }

                GimbalState gs0;
                gs0.aim_id = _control_point0.aim_id;
                gs0.yaw_state.p = 0.0;
                gs0.pitch_state.p = 0.0;
                traj_gs.push_back(gs0, 0.0);
                traj_ap.push_back(_control_point0.aim_point, 0.0);

                for (int i = 1; i <= half; ++i) {
                    if (!push_sample(traj_gs, traj_ap, i * dt, _control_point0)) {
                        return false;
                    }
                }

                return true;
            };
            limit_target_traj_cp0 = control_point0;
            limit_target_traj.clear();
            limit_aim_traj.clear();

            limit_target_traj.reserve(horizon + 1);
            limit_aim_traj.reserve(horizon + 1);

            if (!build_traj(limit_target_traj, limit_aim_traj, limit_target_traj_cp0, horizon, dt)) {
                return cmd;
            }
            limit_target_traj.build_limit(params_.max_yaw_acc, params_.max_pitch_acc, time_in_traj);
        }
        const ControlPoint& target_traj_control_point0 = limit_target_traj_cp0;
        const Trajectory<GimbalState, double>& target_traj =
            static_cast<const Trajectory<GimbalState, double>&>(limit_target_traj);
        auto target_gimbal_state = target_traj.Trajectory::state_at(time_in_traj);
        //目标轨迹，一定击中目标
        auto control = limit_target_traj.LimitTrajectory::state_at(time_in_traj);
        //控制轨迹，轨迹优化后最优控制（并非最优，下位机实际vel acc 可以基于error和上位机规划叠加）
        double control_yaw = angles::normalize_angle(control.yaw_state.p + limit_target_traj_cp0.yaw);
        double control_pitch =
            angles::normalize_angle(control.pitch_state.p + limit_target_traj_cp0.pitch);
        double target_yaw =
            angles::normalize_angle(target_gimbal_state.yaw_state.p + target_traj_control_point0.yaw);
        double target_pitch =
            angles::normalize_angle(target_gimbal_state.pitch_state.p + target_traj_control_point0.pitch);
        cmd.timestamp = Clock::now();
        cmd.yaw = angles::to_degrees(control_yaw);
        cmd.v_yaw = angles::to_degrees(control.yaw_state.v);
        cmd.a_yaw = angles::to_degrees(control.yaw_state.a);
        cmd.pitch = angles::to_degrees(control_pitch);
        cmd.v_pitch = angles::to_degrees(control.pitch_state.v);
        cmd.a_pitch = angles::to_degrees(control.pitch_state.a);
        cmd.target_yaw = angles::to_degrees(target_yaw);
        cmd.target_pitch = angles::to_degrees(target_pitch);
        cmd.fly_time = last_fly_time;
        cmd.appear = true;
        cmd.aim_point = limit_aim_traj.state_at(time_in_traj);
        cmd.aim_point.frame_id = target.get_target_state().frame_id;
        cmd.select_id = now_select;
        auto cal_enbale_diff = [&](double _t) {
            auto aim_point = limit_aim_traj.state_at(_t);
            const double distance = aim_point.pose.translation().norm();
            double half_w = params_.shooting_range_w / 2;
            auto pose1_in_aim_point = ISO3::Identity();
            pose1_in_aim_point.translation() = Vec3(0, -half_w, 0);
            auto pose2_in_aim_point = ISO3::Identity();
            pose2_in_aim_point.translation() = Vec3(0, half_w, 0);
            auto pose1_in_odom = aim_point.pose * pose1_in_aim_point;
            auto pose2_in_odom = aim_point.pose * pose2_in_aim_point;
            auto yaw1 =
                std::atan2(pose1_in_odom.translation().y(), pose1_in_odom.translation().x());
            auto yaw2 =
                std::atan2(pose2_in_odom.translation().y(), pose2_in_odom.translation().x());
            auto aim_yaw =
                std::atan2(aim_point.pose.translation().y(), aim_point.pose.translation().x());
            double shooting_range_yaw = std::min(
                std::abs(angles::normalize_angle(yaw1 - aim_yaw)),
                std::abs(angles::normalize_angle(yaw2 - aim_yaw))
            ); //直接算两个边缘yaw
            double shooting_range_pitch = std::abs(std::atan2(params_.shooting_range_h / 2, distance));
            auto ypr = tools::matrix2euler(aim_point.pose.linear(), tools::EulerOrder::ZYX);
            shooting_range_pitch *= std::cos(ypr[1]);
            shooting_range_yaw = std::max(shooting_range_yaw, angles::from_degrees(params_.min_enable_yaw_deg));
            shooting_range_pitch = std::max(shooting_range_pitch, angles::from_degrees(params_.min_enable_pitch_deg));
            return std::make_pair(std::abs(shooting_range_yaw), std::abs(shooting_range_pitch));
        };
        auto enable_diff = cal_enbale_diff(time_in_traj);
        cmd.enable_yaw_diff = angles::to_degrees(enable_diff.first);
        cmd.enable_pitch_diff = angles::to_degrees(enable_diff.second);
        auto abs_angle_error = [](double from, double to) {
            return std::abs(angles::shortest_angular_distance(from, to));
        };
        cmd.fire_advice = abs_angle_error(angles::from_degrees(cmd.target_yaw), angles::from_degrees(cmd.yaw)) < cmd.enable_yaw_diff
            && abs_angle_error(
                   angles::from_degrees(cmd.target_pitch),
                   angles::from_degrees(cmd.pitch)
               ) < cmd.enable_pitch_diff;
        return cmd;
    }
    bool tick_fire(const RuneTarget& _target, const GimbalCmd& cmd) const noexcept {
        static TimePoint last_fire_time = Clock::now();
        if (_target.voter.mode != motion_model::Voter::Big) {
        }
        if (cmd.appear) {
            if (Clock::now() - last_fire_time > std::chrono::duration<double>(cmd.fly_time * 2.0)) {
                last_fire_time = Clock::now();
                return true;
            }
        }
        return false;
    }
    void set_operator_offset(std::pair<double, double> offset) {
        operator_offset_ = offset;
    }
    std::pair<double, double> get_yaw_pitch_offset() const noexcept {
        return std::make_pair(
            base_yaw_offset_rad + operator_offset_.first, //操作手在线调偏置
            base_pitch_offset_rad + operator_offset_.second
        );
    }
    BallisticTrajectory::Ptr ballistic_trajectory;
    double base_yaw_offset_rad;
    double base_pitch_offset_rad;
    std::pair<double, double> operator_offset_ = std::make_pair(0, 0);
};
RuneAimer::RuneAimer(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
RuneAimer::~RuneAimer() noexcept {}
GimbalCmd RuneAimer::aim(
    const RuneTarget& target,
    double bullet_speed,
    const ISO3& T_shoot2gimbal_abs,
    const ISO3& T_gimbal2gimbal_abs
) {
    return _impl->aim(target, bullet_speed, T_shoot2gimbal_abs, T_gimbal2gimbal_abs);
}
std::pair<double, double> RuneAimer::get_yaw_pitch_offset() {
    return _impl->get_yaw_pitch_offset();
}
void RuneAimer::set_operator_offset(std::pair<double, double> offset) {
    _impl->set_operator_offset(offset);
}
bool RuneAimer::tick_fire(const RuneTarget& _target, const GimbalCmd& cmd) const noexcept {
    return _impl->tick_fire(_target, cmd);
}
} // namespace dts_vision::auto_buff
