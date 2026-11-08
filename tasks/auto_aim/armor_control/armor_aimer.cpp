#include "armor_aimer.hpp"
#include "angles.h"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/dta_utils.hpp"
#include "tools/common/type_common.hpp"
#include "tools/logger.hpp"
#include "tools/utils.hpp"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace dts_vision::auto_aim {
struct ArmorAimer::Impl {
    struct Params {
        double sample_total_time;
        int sample_horizon;
        double fire_delay_min;
        double fire_delay_max;
        double max_yaw_acc;
        double max_pitch_acc;
        double prediction_delay;
        double aim_center_more_prediction_time;
        double shooting_range_h;
        double shooting_range_w_small;
        double shooting_range_w_large;
        double min_enable_pitch_deg;
        double min_enable_yaw_deg;

        void load(const YAML::Node& config) {
            sample_total_time = config["sample_total_time"].as<double>();
            sample_horizon = config["sample_horizon"].as<int>();
            fire_delay_min = config["fire_delay_min"].as<double>();
            fire_delay_max = config["fire_delay_max"].as<double>();
            max_yaw_acc = config["max_yaw_acc"].as<double>();
            max_pitch_acc = config["max_pitch_acc"].as<double>();
            prediction_delay = config["prediction_delay"].as<double>();
            aim_center_more_prediction_time = config["aim_center_more_prediction_time"].as<double>();
            shooting_range_h = config["shooting_range_h"].as<double>();
            shooting_range_w_small = config["shooting_range_w_small"].as<double>();
            shooting_range_w_large = config["shooting_range_w_large"].as<double>();
            min_enable_pitch_deg = config["min_enable_pitch_deg"].as<double>();
            min_enable_yaw_deg = config["min_enable_yaw_deg"].as<double>();
        }
    } params_;
    using MPCType = TinyMpcTrajectory;
    Impl(const YAML::Node& config) {
        params_.load(config);
        ballistic_trajectory = BallisticTrajectory::create(config["ballistic_trajectory"]);
        base_yaw_offset_rad = angles::from_degrees(config["base_yaw_offset"].as<double>());
        base_pitch_offset_rad = angles::from_degrees(config["base_pitch_offset"].as<double>());
        auto type = config["type"].as<std::string>();
        if (type == "mpc" || type == "MPC") {
            MPCType::Params mpc {
                .yaw_max_acc = (float)params_.max_yaw_acc,
                .pitch_max_acc = (float)params_.max_pitch_acc,
                .horizon = params_.sample_horizon,
                .dt = params_.sample_total_time / params_.sample_horizon,
                .max_iter = 50,
            };
            mpc_traj = MPCType::create(mpc);
            DTS_INFO("armor_aimer: using  MpcTrajectory");
        }
    }
    [[nodiscard]] int
    select_armor(const ArmorTarget& target, const AutoAimFsmMode& auto_aim_fsm_mode) const noexcept {
        static int lock_id = -1;
        const auto& target_state = target.get_target_state();
        const auto T_armors2world = target_state.get_armors_pose(target.target_type);
        const int armor_num = static_cast<int>(T_armors2world.size());
        int i_chosen = 0;
        const double center_yaw = std::atan2(target_state.pos().y(), target_state.pos().x());// 是装甲板的偏航角
        auto delta_angle = [&](int i) {
            auto ypr_in_world = tools::matrix2euler(T_armors2world[i].linear(), tools::EulerOrder::ZYX);
            // return angles::normalize_angle(
            //     ypr_in_world[0]
            //     - std::atan2(T_armors2world[i].translation().y(), T_armors2world[i].translation().x())
            // );
            return angles::normalize_angle(ypr_in_world[0] - center_yaw);
        };
        auto abs_delta_angle = [&](int i) { return std::abs(delta_angle(i)); };
        auto pick_best_range = [&](int first, int last, auto&& accept) -> int {
            int best = -1;
            double best_val = std::numeric_limits<double>::infinity();
            for (int i = first; i < last; ++i) {
                if (!accept(i))
                    continue;
                const double val = abs_delta_angle(i);
                if (val < best_val) {
                    best_val = val;
                    best = i;
                }
            }
            return best;
        };
        auto accept_all = [](int) { return true; };
        auto accept_pair = [&](int i) { return (i == 0 || i == 2); };

        if (auto_aim_fsm_mode == AutoAimFsmMode::AIM_SINGLE_ARMOR
            && target.target_type != ArmorType::OUTPOST && armor_num > 0)
        {
            int candidates[2] = { -1, -1 };
            int candidate_count = 0;
            for (int i = 0; i < armor_num && candidate_count < 2; ++i) {
                if (std::abs(delta_angle(i)) <= 60.0 / 57.3) {
                    candidates[candidate_count++] = i;// 选择两个装甲板
                }
            }

            if (candidate_count > 0) {
                int pick = -1;

                if (candidate_count == 1) {
                    pick = candidates[0];
                    lock_id = -1;
                } else {
                    if (lock_id < 0 || (lock_id != candidates[0] && lock_id != candidates[1])) {
                        lock_id = (abs_delta_angle(candidates[0]) < abs_delta_angle(candidates[1]))
                            ? candidates[0]
                            : candidates[1];
                    }
                    pick = lock_id;
                }

                if (pick >= 0 && pick < armor_num) {
                    i_chosen = pick;
                }
            }

            return i_chosen;
        }
        if (armor_num > 0) {
            int best_idx = -1;

            if (auto_aim_fsm_mode == AutoAimFsmMode::AIM_WHOLE_CAR_PAIR //4选2,本质提升控制轨迹与目标轨迹重合窗口
                && target.target_type != ArmorType::OUTPOST)
            {
                best_idx = pick_best_range(0, armor_num, accept_pair);
            }
            if (best_idx < 0) {
                best_idx = pick_best_range(0, armor_num, accept_all);
            }

            i_chosen = best_idx;
        }

        return i_chosen;
    }
    [[nodiscard]] ControlPoint get_control_point(
        const ISO3& T_armor2world,
        const ISO3& T_shoot2gimbal_abs,
        const ISO3& T_gimbal2gimbal_abs,
        double bullet_speed,
        int aim_id
    ) const noexcept {
        ControlPoint control_point;
        auto xyz_in_shoot = T_armor2world.translation() - T_shoot2gimbal_abs.translation();// T_shoot2gimbal_abs=T_shoot2world--->xyz_in_shoot=t_armor2world+t_world2shoot
        auto desired_control_pitch = ballistic_trajectory->solve_pitch(xyz_in_shoot, bullet_speed);
        if (!desired_control_pitch) {
            control_point.valid = false;
            DTS_ERROR(
                "armor_aimer: get_control_point: Failed to solve pitch armor_pos: [{}, {}, {}], bullet_speed: {}",
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
            0.0), tools::EulerOrder::ZYX);// 从当前shoot系转到理想控制系
        const Mat3 R_shoot2gimbal = T_gimbal2gimbal_abs.linear().transpose() * T_shoot2gimbal_abs.linear();// R_shoot2gimbal=R_gimbal_abs2gimbal*R_shoot2gimbal_abs
        auto R_gimbal2control = R_shoot2control * R_shoot2gimbal.transpose();// R_shoot2control*R_gimbal2shoot=R_gimbal2control
        auto ypr_in_gimbal = tools::matrix2euler(R_gimbal2control, tools::EulerOrder::ZYX);
        control_point.yaw = ypr_in_gimbal[0];
        control_point.pitch = ypr_in_gimbal[1];
        control_point.valid = true;

        control_point.aim_point.pose = T_armor2world;
        control_point.aim_id = aim_id;
        return control_point;
    };// 根据当前装甲板位置和速度，计算控制点
    [[nodiscard]] ControlPoint select_and_get_control_point(
        const ArmorTarget& target,
        double bullet_speed,
        const AutoAimFsmMode& fsm,
        const ISO3& T_shoot2gimbal_abs,
        const ISO3& T_gimbal2gimbal_abs
    ) const noexcept {
        const auto& target_state = target.get_target_state();
        const int selected_armor = select_armor(target, fsm);
        auto T_armors2world = target_state.get_armors_pose(target.target_type);
        if (fsm == AutoAimFsmMode::AIM_WHOLE_CAR_CENTER) { //瞄准中间把目标推回原来板子的位置
            auto xyz_in_shoot = target_state.pos() - T_shoot2gimbal_abs.translation();// t_armor2world+t_world2shoot
            double center_xy_dis = std::hypot(xyz_in_shoot.x(), xyz_in_shoot.y());
            double center_yaw = std::atan2(xyz_in_shoot.y(), xyz_in_shoot.x());
            center_xy_dis -= target_state.get_armor_r(selected_armor, target.target_type);// 从车心转到装甲板
            T_armors2world[selected_armor].translation().x() = center_xy_dis * std::cos(center_yaw);// 平移矩阵变成到shoot坐标系下
            T_armors2world[selected_armor].translation().y() = center_xy_dis * std::sin(center_yaw);
        }
        return get_control_point(
            T_armors2world[selected_armor],
            T_shoot2gimbal_abs,
            T_gimbal2gimbal_abs,
            bullet_speed,
            selected_armor
        );
    }// 根据当前装甲板位置和速度，选择一个装甲板，计算控制点
    struct HitCtx {
        ArmorTarget hit_time_target;
        double fly_time;
    };
    std::optional<HitCtx> get_hit(
        const ArmorTarget& target_ready_to_aim,
        double bullet_speed,
        const AutoAimFsmMode& fsm,
        const ISO3& T_shoot2gimbal_abs
    ) const noexcept {
        auto hit_time_target = target_ready_to_aim.fast_copy_without_ekf();// 预瞄装甲板状态
        const int simple_select = select_armor(hit_time_target, fsm);// 这个选板不重要，它主要是为了估算子弹飞行时间
        const auto T_armors2world = hit_time_target.get_target_state().get_armors_pose(hit_time_target.target_type);
        auto prev_pitch_and_fly_time_opt = ballistic_trajectory->solve_pitch_and_flytime(
            T_armors2world[simple_select].translation() - T_shoot2gimbal_abs.translation(),// xyz_in_shoot
            bullet_speed
        );
        if (!prev_pitch_and_fly_time_opt) {
            return std::nullopt;
        }
        auto prev_fly_time = prev_pitch_and_fly_time_opt.value().second;

        for (int iter = 0; iter < 10; ++iter) {
            auto i_target = hit_time_target.fast_copy_without_ekf();
            i_target.set_target_state([&](State& state) {
                state.predict(prev_fly_time, i_target.target_type);
            });
            // auto iter_select = select_armor(i_target, fsm);//不知道哪个最好
            const auto iter_T_armors2world = i_target.get_target_state().get_armors_pose(i_target.target_type);
            // auto iter_pitch_and_fly_time_opt = ballistic_trajectory->solve_pitch_and_flytime(
            //     iter_armors_xyza[iter_select].head<3>(),
            //     bullet_speed
            // );
            auto iter_pitch_and_fly_time_opt = ballistic_trajectory->solve_pitch_and_flytime(
                iter_T_armors2world[simple_select].translation() - T_shoot2gimbal_abs.translation(),// xyz_in_shoot
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
        const double predict_time = prev_fly_time + params_.prediction_delay + (fsm == AutoAimFsmMode::AIM_WHOLE_CAR_CENTER ? params_.aim_center_more_prediction_time : 0); //瞄准中间只能预测一段发弹延迟
        hit_time_target.set_target_state([&](auto& state) {
            state.predict(predict_time, hit_time_target.target_type);// 预测到子弹飞到时刻的目标状态
        });
        return HitCtx {
            .hit_time_target = hit_time_target,
            .fly_time = prev_fly_time,
        };
    }
    /*当前目标状态->先粗选一块板simple_select->用这块板当前位置算飞行时间->把目标预测到子弹飞到时的状态->再用同一块板重新算飞行时间->迭代几次，得到命中时刻的目标状态
HitCtx：如果现在开火，子弹大概多久飞到？飞到的时候，目标车状态是什么样？*/
    struct InputCtx {
        ArmorTarget target;
        double bullet_speed;
        AutoAimFsmMode auto_aim_fsm_mode;
        ISO3 T_shoot2gimbal_abs;
        ISO3 T_gimbal2gimbal_abs;
        bool valid = false;
        bool is_same(
            const ArmorTarget& target_,
            double bullet_speed_,
            AutoAimFsmMode auto_aim_fsm_mode_,
            const ISO3& T_shoot2gimbal_abs_,
            const ISO3& T_gimbal2gimbal_abs_
        ) {
            if (!valid)
                return false;
            bool same_target = target_.this_id == target.this_id;
            return same_target && auto_aim_fsm_mode_ == auto_aim_fsm_mode && std::abs(bullet_speed_ - bullet_speed) < 1e-2
                && T_shoot2gimbal_abs_.isApprox(T_shoot2gimbal_abs, 1e-2)
                && T_gimbal2gimbal_abs_.isApprox(T_gimbal2gimbal_abs, 1e-2);
        }// 判断当前输入和上一次输入是否近似相同
        void set(const ArmorTarget& target_,
            double bullet_speed_,
            AutoAimFsmMode auto_aim_fsm_mode_,
            const ISO3& T_shoot2gimbal_abs_,
            const ISO3& T_gimbal2gimbal_abs_) {
            target = target_.fast_copy_without_ekf();
            bullet_speed = bullet_speed_;
            auto_aim_fsm_mode = auto_aim_fsm_mode_;
            T_shoot2gimbal_abs = T_shoot2gimbal_abs_;
            T_gimbal2gimbal_abs = T_gimbal2gimbal_abs_;
            valid = true;
        }// 保存当前输入状态
    } last_input;
    TimePoint last_time;
    LimitTrajectory limit_target_traj;
    ControlPoint limit_traj_cp0;
    Trajectory<AimPoint, double> limit_aim_traj;
    Trajectory<AimPoint, double> aim_center_aim_traj;
    Trajectory<GimbalState, double> aim_center_target_traj;
    ControlPoint aim_center_traj_cp0;
    double last_fly_time;
    int last_select;
    [[nodiscard]] GimbalCmd aim(
        const ArmorTarget& target_,
        double bullet_speed,
        AutoAimFsmMode fsm,
        const ISO3& T_shoot2gimbal_abs,
        const ISO3& T_gimbal2gimbal_abs
    ) noexcept {
        if (target_.target_type == ArmorType::BASE) {
            fsm = AutoAimFsmMode::AIM_SINGLE_ARMOR;
        }
        GimbalCmd cmd;
        cmd.appear = false;
        bool is_same = last_input.is_same(target_, bullet_speed, fsm, T_shoot2gimbal_abs, T_gimbal2gimbal_abs);
        double time_in_traj = 0.0;
        const auto now = Clock::now();
        if (is_same) { //状态完全未更新，使用第一次构建的轨迹基于时间进行推进减少重复计算
            time_in_traj = std::chrono::duration<double>(now - last_time).count();// 相同的一段轨迹直接沿时间推进就行
        } else {
            last_input.set(target_, bullet_speed, fsm, T_shoot2gimbal_abs, T_gimbal2gimbal_abs);// 保存当前输入状态
            last_time = now;
        }
        auto _target = target_.fast_copy_without_ekf();
        _target.set_target_state([&](State& state) {
            state.predict(now, _target.target_type);
        });// 把目标预测到当前时刻
        auto make_even = [](int x) { return x % 2 == 0 ? x : x + 1; };
        const int horizon = make_even(params_.sample_horizon);// 变成偶数
        const double dt = params_.sample_total_time / horizon;
        auto sample_once = [&](double dt,
                               const ArmorTarget& target,
                               AutoAimFsmMode fsm_mode,
                               const ControlPoint& cp0,
                               GimbalState& out_gimbal_state,
                               AimPoint& out_aim_point) -> bool {
            auto tmp_target = target.fast_copy_without_ekf();// 复制基准目标，避免采样预测影响外部对象
            tmp_target.set_target_state([&](State& state) {// 把采样目标预测到相对时间t
                state.predict(dt, tmp_target.target_type);// 按采样时间偏移更新目标状态，这个t是从target.get_target_state().timestamp开始的相对预测时间，绝对目标状态时刻 = target_state.timestamp + t

            }); //保证轨迹完全为不同时间对同一状态的瞄准控制点
            auto hit_opt = get_hit(tmp_target, bullet_speed, fsm_mode, T_shoot2gimbal_abs);// 计算采样时刻对应的预计命中状态
            if (!hit_opt)// 如果命中预测失败(pitch无效/飞行时间无效)
                return false;
            auto control_point = select_and_get_control_point(
                hit_opt->hit_time_target,// 要打击时刻的target状态
                bullet_speed,
                fsm_mode,
                T_shoot2gimbal_abs,
                T_gimbal2gimbal_abs
            );// 得到打击时刻的瞄准控制点
            if (!control_point.valid)
                return false;
            out_gimbal_state.aim_id = control_point.aim_id;
            out_gimbal_state.yaw_state.p = angles::normalize_angle(control_point.yaw - cp0.yaw);
            out_gimbal_state.pitch_state.p = angles::normalize_angle(control_point.pitch - cp0.pitch);
            out_aim_point = control_point.aim_point;
            return true;
        };// 采样一次目标状态，得到云台状态和瞄准点
        auto push_sample = [&](auto& traj_gimbal_state,
                               auto& traj_aim_point,
                               double dt,
                               const ControlPoint& cp0,
                               AutoAimFsmMode fsm_mode) -> bool {
            GimbalState out_gimbal_state;// 创建本次采样的云台状态输出变量
            AimPoint out_aim_point;// 创建本次采样的瞄准点输出变量
            if (!sample_once(dt, _target, fsm_mode, cp0, out_gimbal_state, out_aim_point)) {// 采样不成功
                return false;
            }
            traj_gimbal_state.push_back(out_gimbal_state, dt);// 把云台状态按时间t插入云台轨迹
            traj_aim_point.push_back(out_aim_point, dt);// 把瞄准点按时间t插入瞄准点轨迹
            return true;
        };// 采样一次目标状态插入轨迹
        if (!is_same) {// 状态已改变，重新构建轨迹
            auto hit_ctx_opt = get_hit(_target, bullet_speed, fsm, T_shoot2gimbal_abs); //直接算到击中目标时间
            if (!hit_ctx_opt) {
                return cmd;
            }
            const auto& hit_ctx = *hit_ctx_opt;
            auto _cp0 = select_and_get_control_point(// 根据零时刻命中状态计算初始控制点
                hit_ctx.hit_time_target,
                bullet_speed,
                fsm,
                T_shoot2gimbal_abs,
                T_gimbal2gimbal_abs
            );
            if (!_cp0.valid) {
                return cmd;
            }
            last_fly_time = hit_ctx.fly_time;
            last_select = _cp0.aim_id;
            auto build_traj = [&](auto& traj_gimbal_state,// 云台轨迹
                                  auto& traj_aim_point,
                                  const ControlPoint& cp0,
                                  AutoAimFsmMode fsm_mode,
                                  int horizon,
                                  double dt) -> bool {
                int half = horizon / 2;
                for (int i = half; i >= 1; --i) {
                    if (!push_sample(traj_gimbal_state, traj_aim_point, -i * dt, cp0, fsm_mode)) {
                        return false;
                    }// 采样插入轨迹不成功
                }// 前半段t-half*dt->t

                GimbalState gs0;
                gs0.aim_id = cp0.aim_id;
                gs0.yaw_state.p = 0.0;
                gs0.pitch_state.p = 0.0;
                traj_gimbal_state.push_back(gs0, 0.0);
                traj_aim_point.push_back(cp0.aim_point, 0.0);

                for (int i = 1; i <= half; ++i) {
                    if (!push_sample(traj_gimbal_state, traj_aim_point, i * dt, cp0, fsm_mode)) {// 采样正时间点并写入轨迹，失败则中止建轨
                        return false;
                    }
                }// 后半段t->t+half*dt

                return true;
            };
            limit_traj_cp0 = _cp0;// 根据零时刻命中状态计算初始控制点
            limit_target_traj.clear();
            limit_aim_traj.clear();
            limit_target_traj.reserve(horizon + 1);// 云台轨迹
            limit_aim_traj.reserve(horizon + 1);

            if (!build_traj(limit_target_traj, limit_aim_traj, limit_traj_cp0, fsm, horizon, dt)) {
                return cmd;
            }
            if (!mpc_traj) {// 如果没有mpc轨迹
                limit_target_traj.build_limit(params_.max_yaw_acc, params_.max_pitch_acc, time_in_traj);
            } else {
                if (!mpc_traj->solve(limit_target_traj, time_in_traj)) {
                    DTS_ERROR("armor_aimer: mpc solve failed");
                    return cmd;
                }
            }

            if (fsm == AutoAimFsmMode::AIM_WHOLE_CAR_CENTER) { //瞄准中间的目标和控制不一样
                aim_center_target_traj.clear();
                aim_center_target_traj.reserve(horizon + 1);
                auto aim_center_target_hit_ctx_opt = get_hit(
                    _target,
                    bullet_speed,
                    AutoAimFsmMode::AIM_WHOLE_CAR_ARMOR,
                    T_shoot2gimbal_abs
                );// 根据零时刻命中状态计算初始控制点
                if (!aim_center_target_hit_ctx_opt) {
                    return cmd;
                }
                const auto& aim_center_target_hit_ctx = *aim_center_target_hit_ctx_opt;
                aim_center_traj_cp0 = select_and_get_control_point(
                    aim_center_target_hit_ctx.hit_time_target,
                    bullet_speed,
                    AutoAimFsmMode::AIM_WHOLE_CAR_ARMOR,
                    T_shoot2gimbal_abs,
                    T_gimbal2gimbal_abs
                );
                aim_center_aim_traj.clear();
                aim_center_aim_traj.reserve(horizon + 1);
                if (!build_traj(
                        aim_center_target_traj,
                        aim_center_aim_traj,
                        aim_center_traj_cp0,
                        AutoAimFsmMode::AIM_WHOLE_CAR_ARMOR,
                        horizon,
                        dt
                    ))
                {
                    return cmd;
                }// 构建轨迹
            }
        } else {// 输入未变化时复用已有轨迹，只向前补新采样点
            // time_in_traj = std::chrono::duration<double>(now - last_time_).count(); // 计算从上次建轨到现在经过了多少秒
            const int target_forward_idx = ((int)(horizon / 2) + time_in_traj / dt);// 计算当前时间对应的最远前向采样索引
            auto replenish_traj = [&](auto& traj_gimbal_state,// 定义补齐已有轨迹前向采样点的lambda
                                      auto& traj_aim_point,// 传入要补齐的瞄准点轨迹
                                      const ControlPoint& cp0,
                                      AutoAimFsmMode fsm_mode,
                                      int target_forward_idx,// 传入目标前向采样索引
                                      double dt) -> bool {
                if (traj_gimbal_state.empty())
                    return false; // 轨迹为空则不补
                int last_forward_idx =// 准备计算当前轨迹最后一个点的采样索引
                    static_cast<int>(std::floor(traj_gimbal_state.time_at(traj_gimbal_state.size() - 1) / dt));
                for (int i = last_forward_idx + 1; i <= target_forward_idx; ++i) {// 从当前轨迹最后一个点之后补到当前需要的前向索引
                    if (!push_sample(traj_gimbal_state, traj_aim_point, i * dt, cp0, fsm_mode)) {// 采样并追加一个新的前向轨迹点
                        return false;
                    }// 采样正时间点并写入轨迹，失败则中止建轨
                }
                return true;// 所有需要的轨迹点补齐成功
            };
            const size_t old_limit_size = limit_target_traj.size();// 记录补点前限制轨迹的长度
            if (!replenish_traj(// 尝试补齐主限制轨迹和瞄准点轨迹
                    limit_target_traj,
                    limit_aim_traj,
                    limit_traj_cp0,
                    fsm,
                    target_forward_idx,// 传入当前需要覆盖到的前向索引
                    dt
                )) {
                return cmd;
            }
            if (limit_target_traj.size() > old_limit_size) {// 如果补点后限制轨迹长度增加(补点成功)
                if (!mpc_traj) {
                    limit_target_traj.update_limit_after_append(// 对新增轨迹段增量更新加速度限制结果
                        params_.max_yaw_acc,
                        params_.max_pitch_acc,
                        time_in_traj,
                        old_limit_size
                    );
                } else {
                    if (!mpc_traj->solve(limit_target_traj, time_in_traj)) {
                        DTS_ERROR("armor_aimer: mpc solve failed");
                        return cmd;
                    }
                }
            }
            if (fsm == AutoAimFsmMode::AIM_WHOLE_CAR_CENTER) { //瞄准中间的目标和控制不一样
                if (!replenish_traj(
                        aim_center_target_traj,
                        aim_center_aim_traj,
                        aim_center_traj_cp0,
                        AutoAimFsmMode::AIM_WHOLE_CAR_ARMOR,
                        target_forward_idx,
                        dt
                    ))
                {
                    return cmd;
                }
            }
        }

        const bool use_center_target = fsm == AutoAimFsmMode::AIM_WHOLE_CAR_CENTER;
        const ControlPoint& target_traj_cp0 = use_center_target ? aim_center_traj_cp0 : limit_traj_cp0;
        const Trajectory<GimbalState, double>& target_traj = use_center_target 
        ? static_cast<const Trajectory<GimbalState, double>&>(aim_center_target_traj) 
        : static_cast<const Trajectory<GimbalState, double>&>(limit_target_traj);
        const Trajectory<AimPoint, double>& aim_traj = use_center_target ? aim_center_aim_traj : limit_aim_traj;
        auto target_gimbal_state = target_traj.Trajectory::state_at(time_in_traj);// 目标轨迹，一定击中目标
        auto get_control = [&](double t) {
            if (!mpc_traj) {
                return limit_target_traj.LimitTrajectory::state_at(t);
            } else {
                return mpc_traj->state_at(t);
            }
        };
        auto control = get_control(time_in_traj);
        //控制轨迹，轨迹优化后最优控制（并非最优，下位机实际vel acc 可以基于error和上位机规划叠加）
        double control_yaw = angles::normalize_angle(control.yaw_state.p + limit_traj_cp0.yaw);// control求出来的是相对当前云台的控制角，所以加上初始角得到绝对角度
        double control_pitch = angles::normalize_angle(control.pitch_state.p + limit_traj_cp0.pitch);
        double target_yaw = angles::normalize_angle(target_gimbal_state.yaw_state.p + target_traj_cp0.yaw);
        double target_pitch = angles::normalize_angle(target_gimbal_state.pitch_state.p + target_traj_cp0.pitch);
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
        cmd.aim_point = aim_traj.state_at(time_in_traj);
        cmd.aim_point.frame_id = _target.get_target_state().frame_id;
        cmd.select_id = last_select;
        const bool is_big = _target.target_type == ArmorType::NO1;
        const double half_w = (is_big ? params_.shooting_range_w_large : params_.shooting_range_w_small) * 0.5;
        const double half_h = params_.shooting_range_h * 0.5;
        const double min_enable_yaw = angles::from_degrees(params_.min_enable_yaw_deg);
        const double min_enable_pitch = angles::from_degrees(params_.min_enable_pitch_deg);
        const double pitch_factor = std::cos(FIFTEEN_DEGREE_RAD);
        auto cal_enbale_diff = [&](double t) {
            auto aim_point = aim_traj.state_at(t);
            const auto& pos = aim_point.pose.translation();
            const double distance = pos.norm();
            auto T_pose12aim_point = ISO3::Identity();
            T_pose12aim_point.translation() = Vec3(0, -half_w, 0);// T_pose12aim_point
            auto T_pose22aim_point = ISO3::Identity();
            T_pose22aim_point.translation() = Vec3(0, half_w, 0);
            auto T_pose12world = aim_point.pose * T_pose12aim_point;// T_aim_point2world * T_pose12aim_point
            auto T_pose22world = aim_point.pose * T_pose22aim_point;
            auto yaw1 = std::atan2(T_pose12world.translation().y(), T_pose12world.translation().x());
            auto yaw2 = std::atan2(T_pose22world.translation().y(), T_pose22world.translation().x());
            auto aim_yaw = std::atan2(pos.y(), pos.x());
            double shooting_range_yaw = std::min(
                std::abs(angles::normalize_angle(yaw1 - aim_yaw)),
                std::abs(angles::normalize_angle(yaw2 - aim_yaw))
            ); //直接算两个边缘yaw
            double shooting_range_pitch = std::abs(std::atan2(half_h, distance));// 半块板的容错
            auto ypr = tools::matrix2euler(aim_point.pose.linear(), tools::EulerOrder::ZYX);
            shooting_range_pitch *= std::cos(ypr[1]); // 俯仰角补偿
            shooting_range_yaw = std::max(shooting_range_yaw, min_enable_yaw);
            shooting_range_pitch = std::max(shooting_range_pitch, min_enable_pitch);
            return std::make_pair(std::abs(shooting_range_yaw), std::abs(shooting_range_pitch));
        };
        auto enable_diff = cal_enbale_diff(time_in_traj);
        cmd.enable_yaw_diff = angles::to_degrees(enable_diff.first);
        cmd.enable_pitch_diff = angles::to_degrees(enable_diff.second);
        auto abs_angle_error = [](double from, double to) {
            return std::abs(angles::shortest_angular_distance(from, to));
        };
        cmd.fire_advice =
            abs_angle_error(angles::from_degrees(cmd.target_yaw), angles::from_degrees(cmd.yaw))
                < cmd.enable_yaw_diff
            && abs_angle_error(
                   angles::from_degrees(cmd.target_pitch),
                   angles::from_degrees(cmd.pitch)
               ) < cmd.enable_pitch_diff;
        if (fsm != AutoAimFsmMode::AIM_WHOLE_CAR_CENTER) {
            auto delay_fire = [&](double delay) {
                auto delay_control = get_control(time_in_traj + delay);
                auto delay_target = target_traj.Trajectory::state_at(time_in_traj + delay);
                auto delay_enable = cal_enbale_diff(time_in_traj + delay);
                const double control_yaw = angles::normalize_angle(delay_control.yaw_state.p + limit_traj_cp0.yaw);
                const double target_yaw = angles::normalize_angle(delay_target.yaw_state.p + target_traj_cp0.yaw);
                const double control_pitch = angles::normalize_angle(delay_control.pitch_state.p + limit_traj_cp0.pitch);
                const double target_pitch = angles::normalize_angle(delay_target.pitch_state.p + target_traj_cp0.pitch);

                return abs_angle_error(control_yaw, target_yaw) < delay_enable.first
                    && abs_angle_error(control_pitch, target_pitch) < delay_enable.second;
            };
            {
                bool has_can = false;

                double t_check = 0 + params_.fire_delay_min; //发射延迟内不让打弹
                while (t_check < (0 + params_.fire_delay_max) && t_check <= horizon / 2.0) {
                    if (!delay_fire(+t_check)) {
                        cmd.no_shoot();
                    } else {
                        has_can = true;
                    }
                    t_check += (dt / 2.0);
                }
                // if (!has_can) {
                //     cmd.no_shoot();
                // }
            }
        }

        return cmd;
    }
    void reset() noexcept {
        last_input.valid = false;
        limit_target_traj.clear();
        limit_aim_traj.clear();
        aim_center_target_traj.clear();
        aim_center_aim_traj.clear();
        if (mpc_traj) mpc_traj->reset();
    }
    void set_operator_offset(std::pair<double, double> offset) {
        operator_offset = offset;
    }
    std::pair<double, double> get_yaw_pitch_offset() const noexcept {
        return std::make_pair(
            base_yaw_offset_rad + operator_offset.first, //操作手在线调偏置
            base_pitch_offset_rad + operator_offset.second
        );
    }
    BallisticTrajectory::Ptr ballistic_trajectory;
    double base_yaw_offset_rad;
    double base_pitch_offset_rad;
    std::pair<double, double> operator_offset = std::make_pair(0, 0);
    MPCType::Ptr mpc_traj;
};
ArmorAimer::ArmorAimer(const YAML::Node& config) {
    _impl = std::make_unique<Impl>(config);
}
ArmorAimer::~ArmorAimer() noexcept {
    _impl.reset();
}
GimbalCmd ArmorAimer::aim(
    const ArmorTarget& target,
    double bullet_speed,
    const AutoAimFsmMode& fsm,
    const ISO3& T_shoot2gimbal_abs,
    const ISO3& T_gimbal2gimbal_abs
) {
    return _impl->aim(target, bullet_speed, fsm, T_shoot2gimbal_abs, T_gimbal2gimbal_abs);
}
std::pair<double, double> ArmorAimer::get_yaw_pitch_offset() {
    return _impl->get_yaw_pitch_offset();
}
void ArmorAimer::set_operator_offset(std::pair<double, double> offset) {
    _impl->set_operator_offset(offset);
}
void ArmorAimer::reset() { _impl->reset(); }
} // namespace dts_vision::auto_aim
