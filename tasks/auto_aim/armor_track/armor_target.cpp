#include "armor_target.hpp"
#include "angles.h"
#include "tasks/auto_aim/armor_track/motion_model.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/common.hpp"
#include "tasks/base/dta_utils.hpp"
#include "tasks/base/web.hpp"
#include "tools/common/type_common.hpp"
#include "tools/logger.hpp"
#include "tools/utils.hpp"
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
namespace dts_vision::auto_aim {
namespace {
    UVLVecZ get_uvl_measurement(
        const cv::Point2f& top,
        const cv::Point2f& bottom,
        const CameraInfo& camera_info
    ) noexcept {
        cv::Point2f measurement_top = top;
        cv::Point2f measurement_bottom = bottom;
        if (MEASURE_NORMALIZED) {
            measurement_top = tools::undistort_point(
                camera_info.camera_matrix,
                camera_info.distortion_coefficients,
                top
            );
            measurement_bottom = tools::undistort_point(
                camera_info.camera_matrix,
                camera_info.distortion_coefficients,
                bottom
            );
        }
        const Eigen::Vector2d top_eigen(measurement_top.x, measurement_top.y);
        const Eigen::Vector2d bottom_eigen(measurement_bottom.x, measurement_bottom.y);
        UVLVecZ observation;
        UVLMeasure::points_to_observation(top_eigen, bottom_eigen, observation.data());
        return observation;
    }

} // namespace

void ArmorTarget::write_log(const char* key) {
    Web::write_log(key, [&](auto& json) {
        json["timestamp"] = static_cast<int>(std::chrono::duration<double>(last_update.time_since_epoch()).count());
        json["target_type"] = string_by_armor_type(target_type);
        json["track_state"] = TrackState::string_by_state(track_state.tracker_state);
        auto& json_target_state = json["target_state"];
        json_target_state["cx"] = Web::val(target_state.pos().x());
        json_target_state["cy"] = Web::val(target_state.pos().y());
        json_target_state["cz"] = Web::val(target_state.pos().z());
        json_target_state["vx"] = Web::val(target_state.vel().x());
        json_target_state["vy"] = Web::val(target_state.vel().y());
        json_target_state["vz"] = Web::val(target_state.vel().z());
        json_target_state["yaw"] = Web::val(target_state.yaw());
        json_target_state["vyaw"] = Web::val(target_state.vyaw());
        json_target_state["r1"] = Web::val(target_state.r1());
        json_target_state["r2"] = Web::val(target_state.r2());
        json_target_state["h"] = Web::val(target_state.h());
        json_target_state["wp"] = Web::val(target_state.w_p());
        json_target_state["wr"] = Web::val(target_state.w_r());
    });
}

void ArmorTarget::reset(
    Armor& armor,
    const ArmorTrackerCfg& config,
    const TimePoint& timestamp,
    int frame_id,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world
) {
    cfg = config;
    target_type = armor.type;
    uvmeasure_ctx = {
        .armor_num = armor_num_by_armor_type(target_type),
        .id = 0,
        .T_camera_cv2world = T_camera_cv2world,
        .camera_info = camera_info.clone(),
            .armor_type = target_type,
        };

    double r_pre;
    Eigen::DiagonalMatrix<double, X_N> p0;
    p0.diagonal().setZero();
    p0.diagonal()[idx::CX] = p0.diagonal()[idx::CY] = p0.diagonal()[idx::CZ] = 1;
    p0.diagonal()[idx::VCX] = p0.diagonal()[idx::VCY] = p0.diagonal()[idx::VCZ] = 10;
    p0.diagonal()[idx::C_ROT_Z] = 1;
    if (use_full_car_rotation(target_type)) {
        // 单块装甲板(135x50mm)对整车俯仰/横滚几乎不可观测(只压缩 50mm 那条边),
        // 给 1rad^2 等于默许初值可以任意翻倒, 这里按 10° 量级给初始不确定度
        constexpr double CAR_TILT_SIGMA = 10.0 * M_PI / 180.0;
        p0.diagonal()[idx::C_ROT_Y] = p0.diagonal()[idx::C_ROT_X] =
            CAR_TILT_SIGMA * CAR_TILT_SIGMA;
    } else {
        // 这两维不参与整车旋转, 协方差置 0 让它们彻底冻结在 0, 不要留给滤波器乱动
        p0.diagonal()[idx::C_ROT_Y] = p0.diagonal()[idx::C_ROT_X] = 0;
    }
    p0.diagonal()[idx::LOG_R1] = p0.diagonal()[idx::LOG_R2] = p0.diagonal()[idx::H] = 1;
    if (target_type == ArmorType::OUTPOST) {
        p0.diagonal()[idx::OUTPOST01DZ] = p0.diagonal()[idx::OUTPOST02DZ] = 1;
    }
    p0.diagonal()[idx::VYAW] = 100;
    if (armor.type == ArmorType::OUTPOST) {
        r_pre = 0.2765;
    } else if (armor.type == ArmorType::BASE) {
        r_pre = 0.3205;
    } else {
        r_pre = 0.25;
    }
    const auto u_q = [] { return Eigen::Matrix<double, X_N, X_N>::Zero(); };
    const auto inject = [](const auto& delta, auto& nominal) { inject_state(delta, nominal); };
    const auto box_minus = [](const auto& nominal, const auto& value, auto& delta) {
        box_minus_state(nominal, value, delta);
    };
    esekf = RobotStateESEKF(
        Predict { .dt = 0.005, .armor_type = target_type },
        u_q,
        inject,
        box_minus,
        p0
    );

    esekf->set_iteration_num(cfg.esekf_iter_num);
    esekf->set_innovation_gate(cfg.innovation_gate_chi2_scale);
    auto T_armor2world = armor.pose;
    auto T_armor2car = ISO3::Identity();
    const double r = r_pre;
    T_armor2car.translation() << -r, 0, 0; //yaw =0(正对)
    const double armor_pitch = (target_type == auto_aim::ArmorType::OUTPOST)
        ? (-auto_aim::FIFTEEN_DEGREE_RAD)
        : (auto_aim::FIFTEEN_DEGREE_RAD);
    T_armor2car.linear() = tools::euler2matrix(Vec3(0, armor_pitch, 0), tools::EulerOrder::ZYX);
    auto T_car2world = T_armor2world * T_armor2car.inverse();
    const Vec3 car_ypr = tools::matrix2euler(T_car2world.linear().eval(), tools::EulerOrder::ZYX);
    target_state.x = Eigen::VectorXd::Zero(X_N);
    target_state.set_pos(T_car2world.translation());
    target_state.x[idx::LOG_R1] = target_state.x[idx::LOG_R2] = std::log(r);
    if (target_type == ArmorType::OUTPOST) {
        target_state.x[idx::OUTPOST01DZ] = target_state.x[idx::OUTPOST02DZ] = 0;
    }
    // 只从单板 PnP 里取偏航作初值: 偏航会压缩 135mm 那条长边, 观测性强;
    // 俯仰/横滚几乎不可观测, 且 IPPE 二义解会直接表现为整车翻倒, 所以初值一律按车体水平给,
    // 真实坡度留给后面有多块板的帧(信息足够时)去收敛。
    // so3_log(Rz(yaw)) == (0, 0, yaw), 因此这里直接写欧拉角的 yaw 即可
    target_state.x[idx::C_ROT_Z] = car_ypr[0];
    target_state.x[idx::C_ROT_Y] = 0;
    target_state.x[idx::C_ROT_X] = 0;
    target_state.timestamp = timestamp;
    target_state.frame_id = frame_id;
    esekf->set_state(target_state.x);
    last_update = timestamp;
    is_inited = true;
    jumped = false;
    last_match_id = -1;
    if (target_type == ArmorType::OUTPOST) {
        outpost_has_all_and_has_set_ids =
            std::make_pair(false, std::vector<bool>(armor_num(), false));
        outpost_has_all_and_has_set_ids->second[0] = true;
    } else {
        outpost_has_all_and_has_set_ids = std::nullopt;
    }
    track_state.reset();
    this_id = GLOBAL_ID++;
    voter.reset(timestamp);
}

bool ArmorTarget::armor_pnp(
    Armor& armor,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world,
    const std::optional<ISO3>& prior_armor2world
) noexcept {
    auto key_points = armor.key_points.landmarks();
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    if (!cv::solvePnPGeneric(
            getArmorKeyPoints3D<cv::Point3f>(armor.type),
            key_points,
            camera_info.camera_matrix,
            camera_info.distortion_coefficients,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE,
            cv::noArray(),
            cv::noArray()
        ))
    {
        return false;
    }

    // IPPE 给出的两组解互为关于视线的镜像: n2 = 2(n1·v)v - n1, 因此 n2·v == n1·v,
    // “正面朝向相机”这个判据对二义性严格不变, 两组解要么同时通过要么同时不通过。
    // 135x50mm 的装甲板在 3m 处两组解的重投影误差差异远小于 1px, 只按误差排序取第一个
    // 等于让像素噪声掷硬币, 而整车三自由度旋转会把这一次掷硬币放大成“整车翻倒”。
    // 这里改成: 先按法向筛掉背对相机的解, 再用先验(跟踪时取预测位姿, 否则取整车直立性)挑选,
    // 重投影误差只在打平时作为次序裁决(solvePnPGeneric 已按误差升序排列)。
    const double armor_pitch_in_car = (armor.type == auto_aim::ArmorType::OUTPOST)
        ? -auto_aim::FIFTEEN_DEGREE_RAD
        : auto_aim::FIFTEEN_DEGREE_RAD;
    // 车体 z 轴在世界系下的方向: R_car2world.col(2) = R_armor2world * Ry(-pitch) * z
    const Vec3 car_up_in_armor(-std::sin(armor_pitch_in_car), 0.0, std::cos(armor_pitch_in_car));

    bool has_valid = false;
    bool has_candidate = false;
    double best_cost = std::numeric_limits<double>::max();
    ISO3 best_pose_in_camera = armor.pose;
    for (size_t i = 0; i < rvecs.size(); ++i) {
        cv::Mat R_cv;
        cv::Rodrigues(rvecs[i], R_cv);
        Mat3 R_armor2camera_cv;// CV->Eigen
        cv::cv2eigen(R_cv, R_armor2camera_cv);
        Vec3 t_armor2camera_cv;
        cv::cv2eigen(tvecs[i], t_armor2camera_cv);
        if (t_armor2camera_cv.z() <= 0) { //解到相机背后
            continue;
        }
        const Vec3 front_normal = -R_armor2camera_cv.col(0);
        const bool front_facing = front_normal.dot(-t_armor2camera_cv) > 0;

        ISO3 T_armor2camera_cv = ISO3::Identity();
        T_armor2camera_cv.translation() = t_armor2camera_cv;
        T_armor2camera_cv.linear() = R_armor2camera_cv;
        const ISO3 T_armor2world_i = T_camera_cv2world * T_armor2camera_cv;

        double cost;
        if (prior_armor2world) {
            // 与先验姿态的角距离, 翻转解会差出几十度, 噪声级别的差异不会误判
            cost = tools::so3_log(
                       (prior_armor2world->linear().transpose() * T_armor2world_i.linear()).eval()
            )
                       .norm();
        } else {
            // 没有先验(新轨迹初始化): 认为车是立在地面上的, 取车体 z 轴最接近世界 +z 的那组解
            cost = -(T_armor2world_i.linear() * car_up_in_armor).z();
        }
        // 背对相机的解只作为兜底, 排在所有正面解之后
        if (!front_facing) {
            cost += 1e3;
        }
        // 严格小于: 打平时保留重投影误差更小的(即更靠前的)那组解
        if (!has_candidate || cost < best_cost - 1e-12) {
            has_candidate = true;
            best_cost = cost;
            best_pose_in_camera = T_armor2camera_cv;
            has_valid = front_facing;
        }
    }
    if (has_candidate) {
        armor.pose = best_pose_in_camera;
    }

    auto T_armor2world = T_camera_cv2world * armor.pose;// T_camera_cv2world=T_camera_cv2world*T_armor2camera_cv
    armor.pose = T_armor2world;
    if (armor.type == auto_aim::ArmorType::OUTPOST || !USE_WROT) { //不考虑整体旋转的目标
        auto ypr_in_world = tools::matrix2euler(armor.pose.linear(), tools::EulerOrder::ZYX);
        auto obj_points = getArmorKeyPoints3D<cv::Point3f>(armor.type);
        const double armor_pitch = (armor.type == auto_aim::ArmorType::OUTPOST)
            ? -auto_aim::FIFTEEN_DEGREE_RAD
            : auto_aim::FIFTEEN_DEGREE_RAD;
        auto center = [](const cv::Point2f& a, const cv::Point2f& b) { return (a + b) * 0.5f; };
        auto eval_yaw = [&](double yaw) -> double {
            auto T_armor2world_ = armor.pose;
            Vec3 search_ypr(yaw, armor_pitch, 0);
            T_armor2world_.linear() = tools::euler2matrix(search_ypr, tools::EulerOrder::ZYX);

            auto T_armor2camera_cv = T_camera_cv2world.inverse() * T_armor2world_;
            auto img_points = tools::reprojection(
                camera_info.camera_matrix,
                camera_info.distortion_coefficients,
                obj_points,
                T_armor2camera_cv
            );

            double error = 0.0; //因为识别角点垂直实际灯条的噪声对姿态影响最大，利用中点误差为其平均误差将其作为约束，这个思想与灯条观测设计思想一致
            error += cv::norm(
                center(
                    img_points[ArmorKeyPointsIndex::LEFT_TOP],
                    img_points[ArmorKeyPointsIndex::RIGHT_TOP]
                )
                - center(
                    key_points[ArmorKeyPointsIndex::LEFT_TOP],
                    key_points[ArmorKeyPointsIndex::RIGHT_TOP]
                )
            );

            error += cv::norm(
                center(
                    img_points[ArmorKeyPointsIndex::LEFT_BOTTOM],
                    img_points[ArmorKeyPointsIndex::RIGHT_BOTTOM]
                )
                - center(
                    key_points[ArmorKeyPointsIndex::LEFT_BOTTOM],
                    key_points[ArmorKeyPointsIndex::RIGHT_BOTTOM]
                )
            );

            return error;
        };
        constexpr double SEARCH_RANGE_DEG = 140.0;
        constexpr double HALF_RANGE_RAD = SEARCH_RANGE_DEG * CV_PI / 180.0 * 0.5;
        double left = ypr_in_world[0] - HALF_RANGE_RAD;
        double right = ypr_in_world[0] + HALF_RANGE_RAD;
        double best_yaw = tools::golden_section_search(
            eval_yaw,
            left,
            right,
            1e-4
        ); //误差天然线性，直接使用黄金分割
        auto best_pose = armor.pose;
        best_pose.linear() = tools::euler2matrix(Vec3(best_yaw, armor_pitch, 0.0), tools::EulerOrder::ZYX);
        armor.pose = best_pose;
    }

    return has_valid;
}

Eigen::Matrix<double, X_N, X_N> ArmorTarget::process_noise(double dt) const noexcept {
    Eigen::Matrix<double, X_N, X_N> q;
    Vec3 q_xyz_body;
    double q_yaw;
    if (target_type == ArmorType::OUTPOST) {
        q_xyz_body = cfg.qxyz_output; // 前哨站车体系加速度方差
        q_yaw = cfg.qyaw_output; // 前哨站角加速度方差
    } else {
        q_xyz_body = cfg.qxyz_common; // 车体系加速度方差
        q_yaw = cfg.qyaw_common; // 角加速度方差
    }
    q.setZero();
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const Mat3 R_car2world = whole_car_pose(target_state.x.data(), target_type).linear(); //认为目标是地面轮式机器人运动，相对自身姿态的z运动强度小
    const Mat3 Q_acc_body = q_xyz_body.asDiagonal();
    const Mat3 Q_acc_world = R_car2world * Q_acc_body * R_car2world.transpose();
    constexpr std::array<int, 3> pos_idx { idx::CX, idx::CY, idx::CZ };
    constexpr std::array<int, 3> vel_idx { idx::VCX, idx::VCY, idx::VCZ };
    constexpr std::array<int, 3> rot_idx { idx::C_ROT_X, idx::C_ROT_Y, idx::C_ROT_Z };
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            q(pos_idx[i], pos_idx[j]) = 0.25 * dt4 * Q_acc_world(i, j);
            q(pos_idx[i], vel_idx[j]) = 0.5 * dt3 * Q_acc_world(i, j);
            q(vel_idx[i], pos_idx[j]) = 0.5 * dt3 * Q_acc_world(i, j);
            q(vel_idx[i], vel_idx[j]) = dt2 * Q_acc_world(i, j);
        }
    }
    q(idx::VYAW, idx::VYAW) += dt2 * q_yaw; //这里将误差设计为相对自身的旋转误差，所以q就是相对自身的运动
    q(idx::C_ROT_Z, idx::VYAW) += 0.5 * dt3 * q_yaw;
    q(idx::VYAW, idx::C_ROT_Z) += 0.5 * dt3 * q_yaw;
    q(idx::C_ROT_Z, idx::C_ROT_Z) += 0.25 * dt4 * q_yaw;
    if (use_full_car_rotation(target_type)) { //只估偏航时不要给这两维灌噪声
        const Mat3 Q_wpr_body = (Vec3(cfg.q_wpr, cfg.q_wpr, 0)).asDiagonal();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                q(rot_idx[i], rot_idx[j]) += dt * Q_wpr_body(i, j);
            }
        }
    }
    q(idx::LOG_R1, idx::LOG_R1) = cfg.q_r / (target_state.r1() * target_state.r1()); //状态量为log
    if (target_type == ArmorType::OUTPOST) {
        q(idx::OUTPOST01DZ, idx::OUTPOST01DZ) = cfg.q_outpost_dz;
        q(idx::OUTPOST02DZ, idx::OUTPOST02DZ) = cfg.q_outpost_dz;
    } else {
        q(idx::LOG_R2, idx::LOG_R2) = cfg.q_r / (target_state.r2() * target_state.r2());
        q(idx::H, idx::H) = cfg.q_h;
    }
    return q;
}

void ArmorTarget::predict_ekf(const TimePoint& timestamp) {
    if (!esekf) {
        throw std::runtime_error("ESEKF is not initialized");
    }
    auto dt = std::chrono::duration<double>(timestamp - target_state.timestamp).count();
    esekf->set_predict_func(Predict { .dt = dt, .armor_type = target_type, .voter = voter });
    esekf->set_update_Q([&]() { return process_noise(dt); });
    target_state.x = esekf->predict();
    target_state.timestamp = timestamp;
    this_id = GLOBAL_ID++;
}

int ArmorTarget::update(
    std::vector<std::pair<int, Armor>>& matched_armors,
    std::vector<std::tuple<int, bool, Light>>& matched_lights,
    const TimePoint& timestamp,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world
) {
    if (matched_armors.empty() && matched_lights.empty()) {
        return 0;
    }
    uvmeasure_ctx.T_camera_cv2world = T_camera_cv2world;

    std::vector<std::shared_ptr<RobotStateESEKF::ObsBase>> obs;

    const auto cal_uvl_residual = [](const Eigen::Matrix<double, UVLZ_N, 1>& z_pred,
                                     const Eigen::Matrix<double, UVLZ_N, 1>& z) {
        return UVLMeasure::residual(z_pred, z);
    };
    const int armors_num = armor_num();
    auto add_uvl_obs = [&](const cv::Point2f& top, const cv::Point2f& bottom, int id, bool is_left
                       ) {
        auto ctx = uvmeasure_ctx;
        ctx.id = id;
        ctx.is_left = is_left;
        ctx.normalized = MEASURE_NORMALIZED;
        const auto observation = get_uvl_measurement(top, bottom, camera_info);
        // Observations evaluate R after add_uva_obs's local key points are destroyed.
        const auto u_r = [this, top, bottom](const Eigen::Matrix<double, UVLZ_N, 1>& z) {
            Eigen::Matrix<double, UVLZ_N, UVLZ_N> r;
            r.setZero();
            auto length = cv::norm(top - bottom); //用灯条长短描述位置误差的强度，可以线性可以log
            // length = std::log1p(length * length);
            const double sigma_px = cfg.r_sigma_px_by_length_ratio * length;
            double sigma_x = sigma_px;
            double sigma_y = sigma_px;
            if (MEASURE_NORMALIZED) {
                sigma_x /= uvmeasure_ctx.camera_info.camera_fx(); //将像素误差转换为归一化坐标误差
                sigma_y /= uvmeasure_ctx.camera_info.camera_fy();
            }
            double sigma_length = cfg.r_sigma_length_by_length_ratio * length;
            if (MEASURE_NORMALIZED) {
                sigma_length /=
                    ((uvmeasure_ctx.camera_info.camera_fx() + uvmeasure_ctx.camera_info.camera_fy())
                     / 2.0);
            }
            double sigma_angle = cfg.r_sigma_angle;
            // sigma_angle *= std::cos(z(idx::UV_ANGLE)
            // ); //越接近垂直上下点的垂直灯条方向偏移越大，这个实际还是需要考虑遮挡模型
            r(idx::UV_ANGLE, idx::UV_ANGLE) = sigma_angle * sigma_angle / 2.0;
            r(idx::UV_CENTER_X, idx::UV_CENTER_X) = sigma_x * sigma_x / 2.0;
            r(idx::UV_CENTER_Y, idx::UV_CENTER_Y) = sigma_y * sigma_y / 2.0;
            r(idx::UV_LENGTH, idx::UV_LENGTH) = sigma_length * sigma_length / 2.0;
            return r;
        };
        UVLMeasure measure { .ctx = ctx };
        obs.push_back(esekf->make_obs(observation, measure, u_r, cal_uvl_residual));
    };

    auto add_uva_obs = [&](Armor& armor, int id) {
        const auto key_points = armor.key_points.landmarks();
        auto ctx = uvmeasure_ctx;
        ctx.id = id;
        add_uvl_obs(
            key_points[ArmorKeyPointsIndex::LEFT_TOP],
            key_points[ArmorKeyPointsIndex::LEFT_BOTTOM],
            id,
            true
        );
        add_uvl_obs(
            key_points[ArmorKeyPointsIndex::RIGHT_TOP],
            key_points[ArmorKeyPointsIndex::RIGHT_BOTTOM],
            id,
            false
        );
        // 已经在跟踪的轨迹用预测位姿当先验去消 IPPE 二义性, 否则(DETECTING)退回整车直立性判据
        const std::optional<ISO3> pnp_prior = track_state.is_tracking()
            ? std::optional<ISO3>(armor_pose(target_state.x.data(), id, armors_num, target_type))
            : std::nullopt;
        if (matched_armors.size() == 1
            && armor_pnp(armor, camera_info, T_camera_cv2world, pnp_prior))
        { //单个装甲板容易退化，使状态被时间误导，这里用ippe解出的深度差进行约束
            auto T_armor2camera_cv = T_camera_cv2world.inverse() * armor.pose; //转回相机
            auto t_keypoints2armor = getArmorKeyPoints3D<Vec3>(armor.type);// 装甲自身坐标系里的四个角点的三维坐标
            auto t_keypoint2camera_cv = [&](ArmorKeyPointsIndex index) -> Vec3 {
                const auto& t_keypoint2armor = t_keypoints2armor[std::to_underlying(index)];
                return T_armor2camera_cv * t_keypoint2armor;// Eigen::Isometry3d对Vector3d重载了operator*，实际执行的是：T.linear() * p + T.translation()
            };
            const Vec3 t_left_center2camera_cv = (t_keypoint2camera_cv(ArmorKeyPointsIndex::LEFT_TOP)
                                      + t_keypoint2camera_cv(ArmorKeyPointsIndex::LEFT_BOTTOM))
                / 2.0;
            const Vec3 t_right_center2camera_cv = (t_keypoint2camera_cv(ArmorKeyPointsIndex::RIGHT_TOP)
                                       + t_keypoint2camera_cv(ArmorKeyPointsIndex::RIGHT_BOTTOM))
                / 2.0;
            // const Vec3 top_center = (t_keypoint2camera_cv(ArmorKeyPointsIndex::LEFT_TOP)
            //                           + t_keypoint2camera_cv(ArmorKeyPointsIndex::RIGHT_TOP))
            //     / 2.0;
            // const Vec3 bottom_center = (t_keypoint2camera_cv(ArmorKeyPointsIndex::LEFT_BOTTOM)
            //                              + t_keypoint2camera_cv(ArmorKeyPointsIndex::RIGHT_BOTTOM))
            //     / 2.0;
            const double lr_depth_diff = t_left_center2camera_cv.z() - t_right_center2camera_cv.z();// camer_cv系中z朝前，左右灯条深度差
            // const double tb_depth_diff = top_center.z() - bottom_center.z();
            DiffVecZ observation_diff;
            observation_diff << lr_depth_diff;
            DiffMeasure measure_diff { .ctx = ctx };
            obs.push_back(esekf->make_obs(
                observation_diff,
                measure_diff,
                [&](const DiffVecZ& z) {
                    Eigen::Matrix<double, DIFFZ_N, DIFFZ_N> r;
                    r.setZero();
                    auto r_sigma = cfg.r_sigma_armor_lights_depth_diff;
                    // / std::abs(z[0] / getArmorWH(target_type).first); //实际触发到这里的情况深度差都不大，这么映射太猛了
                    r(0, 0) = r_sigma * r_sigma / 2.0;
                    // r(1, 1) = r_sigma * r_sigma / 2.0;
                    return r;
                },
                [](const DiffVecZ& z_pred, const DiffVecZ& z) {
                    return DiffMeasure::residual(z_pred, z);
                }
            ));
        }
    };

    auto update_outpost_state = [&](int id) {
        if (!outpost_has_all_and_has_set_ids || outpost_has_all_and_has_set_ids->first) {
            return;
        }
        auto& has_ids = outpost_has_all_and_has_set_ids->second;
        has_ids[id] = true;
        if (std::all_of(has_ids.begin(), has_ids.end(), [](bool v) { return v; })) {
            outpost_has_all_and_has_set_ids->first = true;
        }
    };

    int accepted = 0;
    std::vector<bool> used_id(armor_num(), false);
    {
        for (auto& [id, armor]: matched_armors) {
            jumped |= (id != 0);
            update_outpost_state(id);
            last_match_id = id;
            used_id[id] = true;
            add_uva_obs(armor, id);
        }
    }

    for (const auto& [id, is_left, light]: matched_lights) {
        add_uvl_obs(light.top, light.bottom, id, is_left);
    }
    if (!obs.empty()) {
        // 整车俯仰/横滚的信息门控: 只有同时看到两块以上不同编号的板(相邻板相差 90°)时,
        // 观测才真正约束得住面外俯仰; 单板帧把这两维的增益置零, 只更新位置/偏航/半径,
        // 否则 50mm 短边上的像素噪声会被当成整车倾角, 一路积累成俯视姿态。
        const int observed_id_num =
            static_cast<int>(std::count(used_id.begin(), used_id.end(), true));
        VecX gain_mask = VecX::Ones();
        if (use_full_car_rotation(target_type) && observed_id_num < 2) {
            gain_mask[idx::C_ROT_X] = 0.0;
            gain_mask[idx::C_ROT_Y] = 0.0;
        }
        esekf->set_gain_mask(gain_mask);

        target_state.x = esekf->update_multi(obs);
        accepted = static_cast<int>(obs.size()) - esekf->last_rejected_obs_num();

        // 整帧观测都被新息门控拒掉时, update_multi 原样返回预测值, 这一帧等价于没看到目标。
        // 此时不刷新 last_update, 让上层 FSM 照常走 TEMP_LOST / LOST ——
        // 否则一条持续被门控拒绝的轨迹会一直靠预测空转, 永远不超时。
        if (accepted > 0) {
            if (clamp_car_tilt(target_state.x, target_type)) { //兜底硬约束, 不让整车被拧倒
                esekf->set_state(target_state.x);
            }
            target_state.timestamp = timestamp;
            last_update = timestamp;
            this_id = GLOBAL_ID++;
            voter.update(target_state.yaw(), timestamp);
        }
    }
    update_count += accepted;
    return accepted;
}
std::vector<std::pair<int, Armor>> ArmorTarget::match_armor(
    std::vector<Armor>& armors,
    const TimePoint& timestamp,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world
) const noexcept {
    constexpr double MAX_COST = 1e9;
    std::vector<std::pair<int, Armor>> result;
    const int n_obs = static_cast<int>(armors.size());
    auto pred_state = target_state;// 预测状态
    pred_state.predict(timestamp, target_type);
    const int armors_num = armor_num();
    std::vector<int> maybe_visible;
    std::vector<std::pair<double, int>> angle_dis_in_camera_cv;
    for (int i = 0; i < armors_num; i++) {
        auto T_armor2world = armor_pose(pred_state.x.data(), i, armors_num, target_type);
        auto T_armor2camera_cv = T_camera_cv2world.inverse() * T_armor2world;
        Vec3 axis_x = T_armor2camera_cv.linear().col(0);// 旋转矩阵第0列，即装甲板坐标系的x轴在相机系下的方向(向前)
        Vec3 front_normal = -axis_x;// 取反
        auto normal = T_armor2camera_cv.linear().col(0);
        angle_dis_in_camera_cv.emplace_back(front_normal.dot(-T_armor2camera_cv.translation()), i);// 计算夹角偏差并存入
    }// front_normal.dot(-T_armor2camera_cv.translation()):装甲板正面法向与装甲板→相机反向(相机->装甲板(x前正y左正z上正)，但是坐标系的方向还是装甲板的,这样可以看做坐标系取反(x后正y右正z下正)(注意不是变成了装甲板的坐标系方向))的点积
    std::sort(
        angle_dis_in_camera_cv.begin(),
        angle_dis_in_camera_cv.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; }
    );// 正对相机的装甲板在前面
    const auto visible_count = std::min<std::size_t>(
        3,
        angle_dis_in_camera_cv.size()
    ); //前3个和正对差小的,0/1/2/3
    for (std::size_t i = 0; i < visible_count; ++i) {
        maybe_visible.push_back(angle_dis_in_camera_cv[i].second);
    }
    const bool all_init = outpost_has_all_and_has_set_ids ? outpost_has_all_and_has_set_ids->first : jumped;
    std::vector<std::vector<double>> cost(
        n_obs,
        std::vector<double>(maybe_visible.size(), MAX_COST + 1)
    );
    std::vector<std::array<cv::Point2f, 4>> meas_list(n_obs);

    for (int j = 0; j < n_obs; ++j) {
        auto key_points = armors[j].key_points.landmarks();

        meas_list[j] = { key_points[ArmorKeyPointsIndex::LEFT_TOP],
                         key_points[ArmorKeyPointsIndex::RIGHT_TOP],
                         key_points[ArmorKeyPointsIndex::RIGHT_BOTTOM],
                         key_points[ArmorKeyPointsIndex::LEFT_BOTTOM] };
    }
    for (int j = 0; j < n_obs; ++j) {
        bool in_gate = false;
        double min_total_cost = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < maybe_visible.size(); ++i) { //抽象为4边形和4边形匹配
            const int id = maybe_visible[i];
            const auto left_light = predict_light(id, true, pred_state, camera_info, T_camera_cv2world);
            const auto right_light = predict_light(id, false, pred_state, camera_info, T_camera_cv2world);
            std::array<cv::Point2f, 4> pred = { left_light.first,
                                                right_light.first,
                                                right_light.second,
                                                left_light.second };
            const auto& meas = meas_list[j];

            cv::Point2f cp(0, 0), cm(0, 0);

            for (int k = 0; k < 4; ++k) {
                cp += pred[k];
                cm += meas[k];
            }
            cp *= 0.25f;
            cm *= 0.25f;
            double center_err = cv::norm(cp - cm); //整体位置误差
            auto angle = [&](const cv::Point2f& p1, const cv::Point2f& p2) {
                return std::atan2(p2.y - p1.y, p2.x - p1.x);
            };
            double angle_err = 0; //每条边的角度差，表现姿态
            for (int k = 0; k < 4; ++k) {
                angle_err += std::abs(angles::normalize_angle(
                    angle(pred[k], pred[(k + 1) % 4]) - angle(meas[k], meas[(k + 1) % 4])
                ));
            }
            double side_length_err = 0;
            double side_length_p = 0;
            double side_length_m = 0;
            for (int k = 0; k < 4; ++k) {
                side_length_m += cv::norm(meas[k] - meas[(k + 1) % 4]);// 计算测量的边长
                side_length_p += cv::norm(pred[k] - pred[(k + 1) % 4]);// 计算预测的边长
            }
            side_length_err = std::abs(side_length_p - side_length_m) / side_length_p; //边长差，表现距离缩放
            double total_cost = cfg.armor_match_w_center_err * center_err
                + cfg.armor_match_w_angle_err * angle_err
                + cfg.armor_match_w_side_length_err * side_length_err;
            if (std::isfinite(total_cost)
                && total_cost < (!all_init ? cfg.armor_match_gate_not_all_init : cfg.armor_match_gate))
            {
                cost[j][i] = total_cost;
                in_gate = true;
            }
            if (total_cost < min_total_cost) {
                min_total_cost = total_cost;
            }
        }
        if (!in_gate) {
            DTS_WARN("match out of gate min total cost: {}", min_total_cost);
        }
    }
    for (auto [obs, i]: greedy_match(cost, n_obs, maybe_visible.size(), MAX_COST)) {
        result.emplace_back(maybe_visible[i], armors[obs]);
    }
    return result;
}
std::pair<cv::Point2f, cv::Point2f> ArmorTarget::predict_light(
    int armor_id,
    bool is_left,
    const State& state,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world
) const noexcept {
    UVCtx ctx { .armor_num = armor_num(),
                .id = armor_id,
                .T_camera_cv2world = T_camera_cv2world,
                .camera_info = camera_info,
                .armor_type = target_type,
                .is_left = is_left };
    UVLMeasure measure { .ctx = ctx };
    return measure.projected_points(state.x);
}
std::vector<std::tuple<int, bool, Light>> ArmorTarget::match_light(
    std::vector<Light>& lights,
    const std::vector<std::pair<int, Armor>>& matched_armors,
    const TimePoint& timestamp,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world
) const noexcept {
    if (!cfg.enable_lights_measure || target_type == ArmorType::BASE || matched_armors.empty()) {
        return {};
    }
    auto pred_state = target_state;
    pred_state.predict(timestamp, target_type);
    const int armors_num = armor_num();
    std::vector<bool> matched(armors_num, false);
    for (auto& [id, _]: matched_armors) {
        matched[id] = true;
    }
    std::vector<std::pair<double, int>> angle_dis_in_camera_cv;
    for (int i = 0; i < armors_num; i++) {
        auto T_armor2world = armor_pose(pred_state.x.data(), i, armors_num, target_type);
        auto T_armor2camera_cv = T_camera_cv2world.inverse() * T_armor2world;
        Vec3 axis_x = T_armor2camera_cv.linear().col(0);
        Vec3 front_normal = -axis_x;
        auto normal = T_armor2camera_cv.linear().col(0);
        angle_dis_in_camera_cv.emplace_back(front_normal.dot(-T_armor2camera_cv.translation()), i);
    }
    std::sort(
        angle_dis_in_camera_cv.begin(),
        angle_dis_in_camera_cv.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; }
    );
    if (angle_dis_in_camera_cv.empty()) {
        return {};
    }
    std::vector<std::tuple<int, bool, std::pair<cv::Point2f, cv::Point2f>>> visible_lights;
    auto maybe_visible = [&](int armor_id, bool is_left) {
        if (matched[armor_id]) {
            return;
        }
        visible_lights.emplace_back(
            armor_id,
            is_left,
            predict_light(armor_id, is_left, pred_state, camera_info, T_camera_cv2world)
        );
    };
    const auto closest = std::min_element(
        angle_dis_in_camera_cv.begin(),
        angle_dis_in_camera_cv.end(),
        [](const auto& a, const auto& b) {
            return a.first > b.first;
        } //最正对那个，还是需要遮挡模型更好
    );
    maybe_visible((closest->second + armors_num - 1) % armors_num, false); //左边板子右灯条
    maybe_visible((closest->second + 1) % armors_num, true); // 右边板子左灯条
    maybe_visible(closest->second, false);
    maybe_visible(closest->second, true);
    return match_light(lights, matched_armors, visible_lights);
}
std::vector<std::tuple<int, bool, Light>> ArmorTarget::match_light(
    std::vector<Light>& lights,
    const std::vector<std::pair<int, Armor>>& matched_armors,
    const std::vector<std::tuple<int, bool, std::pair<cv::Point2f, cv::Point2f>>>& visible_lights
) const noexcept {
    std::vector<std::tuple<int, bool, Light>> result;
    if (!cfg.enable_lights_measure || visible_lights.empty()) {
        return result;
    }
    const int n_obs = static_cast<int>(lights.size());
    constexpr double MAX_COST = 1e9;
    std::vector<std::vector<double>> cost(
        n_obs,
        std::vector<double>(visible_lights.size(), MAX_COST + 1)
    );
    auto calc_cost = //灯条没有数字特征，这里严格进行门控，使用位置匹配
        [&](const Light& light,
            const std::tuple<int, bool, std::pair<cv::Point2f, cv::Point2f>>& visible_light
        ) -> double {
        const auto& pred = std::get<2>(visible_light);
        const double pred_len = cv::norm(pred.first - pred.second);
        if (!std::isfinite(pred_len) || pred_len <= 0.0 || !std::isfinite(light.length)
            || light.length <= 0.0) {
            return MAX_COST + 1;
        }
        const double len_err = std::abs(light.length - pred_len);
        if (len_err > pred_len * cfg.light_match_length_ratio_gate) {
            return MAX_COST + 1;
        }

        const double pred_angle = std::atan2(pred.first.x - pred.second.x, pred.first.y - pred.second.y);
        const double light_angle = std::atan2(light.top.x - light.bottom.x, light.top.y - light.bottom.y);// 灯条的x，y是像素坐标，x左右，y上下
        const double angle_err = std::abs(angles::normalize_angle(light_angle - pred_angle));
        if (angle_err > cfg.light_match_angle_gate) {
            return MAX_COST + 1;
        }

        const double pos_err =
            cv::norm(light.top - pred.first) + cv::norm(light.bottom - pred.second);
        if (pos_err > pred_len * cfg.light_match_pos_gate_by_length_ratio) {
            return MAX_COST + 1;
        }

        return pos_err;
    };

    for (int j = 0; j < n_obs; ++j) {
        for (std::size_t i = 0; i < visible_lights.size(); ++i) {
            const int armor_id = std::get<0>(visible_lights[i]);
            if (std::any_of(matched_armors.begin(), matched_armors.end(),
                            [armor_id](const auto& armor) { return armor.first == armor_id; })) {
                continue;
            }
            cost[j][i] = calc_cost(lights[j], visible_lights[i]);
        }
        // A detected light closer to an already measured edge must not be reassigned
        // to an adjacent plate merely because that plate is still a candidate.
        const double best_cost = *std::min_element(cost[j].begin(), cost[j].end());
        for (const auto& [armor_id, armor]: matched_armors) {
            auto key_points = armor.key_points;
            const auto points = key_points.landmarks();
            const double left_cost = calc_cost(lights[j], { armor_id, true,
                { points[ArmorKeyPointsIndex::LEFT_TOP], points[ArmorKeyPointsIndex::LEFT_BOTTOM] } });
            const double right_cost = calc_cost(lights[j], { armor_id, false,
                { points[ArmorKeyPointsIndex::RIGHT_TOP], points[ArmorKeyPointsIndex::RIGHT_BOTTOM] } });
            if (std::min(left_cost, right_cost) <= best_cost) {
                std::fill(cost[j].begin(), cost[j].end(), MAX_COST + 1);
                break;
            }
        }
    }

    for (auto [obs, id]: greedy_match(cost, n_obs, visible_lights.size(), MAX_COST)) {
        const auto& [armor_id, is_left, _] = visible_lights[id];
        lights[obs].laji = false;
        result.emplace_back(armor_id, is_left, lights[obs]);
    }
    return result;
}
[[nodiscard]] cv::Rect ArmorTarget::get_net_focus_roi(
    const TimePoint& timestamp,
    const ISO3& T_camera_cv2world,
    const CameraInfo& camera_info,
    const cv::Size& image_size,
    double target_wh_ratio
) const noexcept {
    if (!need_focus()) {
        return cv::Rect(0, 0, image_size.width, image_size.height);
    }

    cv::Rect rect = expanded(timestamp, T_camera_cv2world, camera_info, image_size);
    constexpr double expand_ratio = 1.4;
    constexpr double expand_ratio_base = 3.0;
    rect = tools::expand_and_clip_rect(
        rect,
        (target_type == ArmorType::BASE) ? expand_ratio_base : expand_ratio,
        image_size
    );
    const cv::Rect img_rect(0, 0, image_size.width, image_size.height);

    if ((rect & img_rect).area() <= 0) {
        return img_rect;
    }

    rect &= img_rect;

    double rect_w = std::max<double>(rect.width, 1.0);
    double rect_h = std::max<double>(rect.height, 1.0);
    double ratio = (std::isfinite(target_wh_ratio) && target_wh_ratio > 0.0) ? target_wh_ratio : 1.0;

    double target_w = rect_w;
    double target_h = rect_h;
    if (target_w / target_h < ratio) {
        target_w = target_h * ratio;
    } else {
        target_h = target_w / ratio;
    }

    double cx = rect.x + rect.width / 2.0;
    double cy = rect.y + rect.height / 2.0;
    cv::Rect ratio_rect(
        static_cast<int>(cx - target_w / 2.0),
        static_cast<int>(cy - target_h / 2.0),
        static_cast<int>(target_w),
        static_cast<int>(target_h)
    );
    ratio_rect &= img_rect;

    double dt = std::chrono::duration<double>(timestamp - last_update).count();
    double lost_dt = cfg.lost_time_thres;
    const double dt_clamped = std::clamp(dt, 0.0, lost_dt);

    int base_side = std::max(ratio_rect.width, ratio_rect.height);
    int max_side = std::max(image_size.width, image_size.height);
    int side = static_cast<int>(base_side + (max_side - base_side) * (dt_clamped / lost_dt));
    if (dt >= lost_dt)
        side = max_side;

    int square_cx = ratio_rect.x + ratio_rect.width / 2;
    int square_cy = ratio_rect.y + ratio_rect.height / 2;
    cv::Rect square(square_cx - side / 2, square_cy - side / 2, side, side);
    square &= img_rect;

    return square;
}
[[nodiscard]] cv::Rect ArmorTarget::expanded(
    const TimePoint& timestamp,
    const ISO3& T_camera_cv2world,
    const CameraInfo& camera_info,
    const cv::Size& image_size
) const noexcept {
    auto pts = expanded_pts(timestamp, T_camera_cv2world, camera_info);
    cv::Rect rect = cv::boundingRect(pts);
    const cv::Rect img_rect = cv::Rect(0, 0, image_size.width, image_size.height);
    if ((rect & img_rect).area() <= 0) {
        return img_rect;
    }
    return rect;
}
[[nodiscard]] std::vector<cv::Point2f> ArmorTarget::expanded_pts(
    const TimePoint& timestamp,
    const ISO3& T_camera_cv2world,
    const CameraInfo& camera_info
) const noexcept {
    std::vector<cv::Point2f> pts;
    auto tmp_target_state = target_state;
    tmp_target_state.predict(timestamp, target_type);
    const int armors_num = armor_num();
    pts.reserve(armors_num * 4);
    for (int id = 0; id < armors_num; ++id) {
        for (const bool is_left: { true, false }) {
            const auto light =
                predict_light(id, is_left, tmp_target_state, camera_info, T_camera_cv2world);
            pts.push_back(light.first);
            pts.push_back(light.second);
        }
    }
    return pts;
}
} // namespace dts_vision::auto_aim
