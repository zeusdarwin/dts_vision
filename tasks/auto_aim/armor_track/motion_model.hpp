#pragma once
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/common.hpp"
#include "tools/common/type_common.hpp"
#include "tools/kalmanlib/error_state_extended_kalman_filter.hpp"
#include "tools/utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <chrono>
#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>
namespace dts_vision::auto_aim {

namespace idx {
    enum { CX, VCX, CY, VCY, CZ, VCZ, C_ROT_Z, VYAW, LOG_R1, P1, P2, C_ROT_Y, C_ROT_X, X_N };
    constexpr int LOG_R2 = P1;
    constexpr int H = P2;
    constexpr int OUTPOST01DZ = P1;
    constexpr int OUTPOST02DZ = P2;
    enum { YPD_Y, YPD_P, YPD_D, A_ROT_X, A_ROT_Y, A_ROT_Z, _YPD_Z_N };
    // enum { YPD_Y, YPD_P, YPD_D, A_ROT_YAW, _YPD_Z_N };
    enum { UV_ANGLE, UV_CENTER_X, UV_CENTER_Y, UV_LENGTH, _UVLZ_N };
} // namespace idx
constexpr int X_N = idx::X_N;
constexpr int UVLZ_N = idx::_UVLZ_N;
constexpr double OUTPOST_R = 0.55 / 2.0;
constexpr double OUTPOST_WZ = 2.51;
constexpr double MIN_ARMOR_R = 0.05;
constexpr double MAX_ARMOR_R = 1.0;
constexpr int YPDZ_N = idx::_YPD_Z_N;
using VecX = Eigen::Matrix<double, X_N, 1>;
using UVLVecZ = Eigen::Matrix<double, UVLZ_N, 1>;
using YPDVecZ = Eigen::Matrix<double, YPDZ_N, 1>;
constexpr int DIFFZ_N = 1;
using DiffVecZ = Eigen::Matrix<double, DIFFZ_N, 1>;
// 整车是否估计三自由度旋转。false 时只估偏航(整车直立), C_ROT_X/C_ROT_Y 退化为常 0:
// 单块装甲板只有 50mm 的短边承载面外俯仰信息, 这两维近似不可观测, 估了只会让滤波器发散;
// 平地上的车本来也不存在整车俯仰/横滚。需要打坡上的车时再打开, 并配合
// armor_pnp 的先验选解 + 俯仰/横滚信息门控 + clamp_car_tilt 一起用。
constexpr bool USE_WROT = false;
template<typename T>
inline T normalize_angle(T a) {// 
    const T two_pi = T(2.0 * M_PI);
    return a - two_pi * ceres::floor((a + T(M_PI)) / two_pi);
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> car_rotation(const T x[X_N], ArmorType armor_type) {
    if (armor_type == ArmorType::OUTPOST || armor_type == ArmorType::BASE || !USE_WROT) {
        Eigen::Matrix<T, 3, 1> yaw_rotvec;
        yaw_rotvec << T(0), T(0), x[idx::C_ROT_Z];// 只有绕z轴的旋转
        return tools::so3_exp(yaw_rotvec);
    }
    return tools::so3_exp(Eigen::Matrix<T, 3, 1>(x[idx::C_ROT_X], x[idx::C_ROT_Y], x[idx::C_ROT_Z])
    );
}
//! 该目标是否使用整车三自由度旋转(与 car_rotation 的分支保持一致)。
//! 前哨站/基地只用偏航, 它们的 C_ROT_X/C_ROT_Y 不参与旋转, 不能对其做姿态级的修正。
inline bool use_full_car_rotation(ArmorType armor_type) noexcept {
    return USE_WROT && armor_type != ArmorType::OUTPOST && armor_type != ArmorType::BASE;
}
//! 整车倾角上限: 地面轮式机器人就算在坡上也不会倾到这个角度以上。
constexpr double MAX_CAR_TILT = 25.0 * M_PI / 180.0;
//! 把车体 z 轴与世界 z 轴的夹角硬约束在 MAX_CAR_TILT 以内, 返回是否做了修正。
//! 偏航被完整保留(修正量绕水平轴), 只把多余的倾角转回去。
template<class StateVector>
inline bool clamp_car_tilt(StateVector& x, ArmorType armor_type) noexcept {
    if (!use_full_car_rotation(armor_type)) {
        return false;
    }
    const Mat3 R_car2world =
        tools::so3_exp(Vec3(x[idx::C_ROT_X], x[idx::C_ROT_Y], x[idx::C_ROT_Z]));
    const Vec3 car_up = R_car2world.col(2);
    const double tilt = std::acos(std::clamp(car_up.z(), -1.0, 1.0));
    if (!(tilt > MAX_CAR_TILT)) { //含 NaN 保护
        return false;
    }
    Vec3 axis = car_up.cross(Vec3::UnitZ());
    const double axis_norm = axis.norm();
    if (axis_norm < 1e-9) { //与 ±z 共线, 无法定义修正轴
        return false;
    }
    axis /= axis_norm;
    const Vec3 clamped =
        tools::so3_log((tools::so3_exp((axis * (tilt - MAX_CAR_TILT)).eval()) * R_car2world).eval());
    x[idx::C_ROT_X] = clamped.x();
    x[idx::C_ROT_Y] = clamped.y();
    x[idx::C_ROT_Z] = clamped.z();
    return true;
}
template<typename T>
inline T armor_radius(const T x[X_N], int id, int armor_num, ArmorType armor_type) {
    if (armor_type == ArmorType::BASE) {
        return T(0.0);
    }
    const bool is_r2 = (armor_num == 4) && (id & 1);
    return ceres::exp(is_r2 ? x[idx::LOG_R2] : x[idx::LOG_R1]);
}
template<typename T>
inline Eigen::Transform<T, 3, Eigen::Isometry>
whole_car_pose(const T x[X_N], ArmorType armor_type) {
    Eigen::Transform<T, 3, Eigen::Isometry> T_car2world =
        Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    T_car2world.translation() << x[idx::CX], x[idx::CY], x[idx::CZ];
    T_car2world.linear() = car_rotation(x, armor_type);
    return T_car2world;
}
template<typename T>
inline Eigen::Transform<T, 3, Eigen::Isometry>
armor_pose(const T x[X_N], int id, int armor_num, ArmorType armor_type) {
    const T yaw = normalize_angle(T(id) * T(2.0 * M_PI / armor_num));
    const bool outpost = armor_type == ArmorType::OUTPOST;
    const bool is_r2 = (armor_num == 4) && (id & 1);
    const T r = armor_radius(x, id, armor_num, armor_type);
    const T armor_x = -ceres::cos(yaw) * r;
    const T armor_y = -ceres::sin(yaw) * r;
    T armor_z = T(0);
    if (outpost) {
        if (id == 1)
            armor_z = x[idx::OUTPOST01DZ];
        else if (id == 2)
            armor_z = x[idx::OUTPOST02DZ];
    } else {
        armor_z = is_r2 ? x[idx::H] : T(0);
    }
    auto T_armor2car = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    T_armor2car.translation() << armor_x, armor_y, armor_z;

    const T armor_pitch = outpost ? T(-FIFTEEN_DEGREE_RAD) : T(FIFTEEN_DEGREE_RAD);
    T_armor2car.linear() = tools::euler2matrix(Eigen::Vector3<T>(yaw, armor_pitch, T(0)), tools::EulerOrder::ZYX);
    return whole_car_pose(x, armor_type) * T_armor2car;// T_car2world * T_armor2car = T_armor2world
}

template<class StateVector>
inline auto state_rotation(const StateVector& state) {
    using Scalar = typename std::decay_t<StateVector>::Scalar;
    using Vec3T = Eigen::Matrix<Scalar, 3, 1>;
    return tools::so3_exp(Vec3T(state[idx::C_ROT_X], state[idx::C_ROT_Y], state[idx::C_ROT_Z]));
}

template<class DeltaVector, class StateVector>
inline void inject_state(const DeltaVector& delta, StateVector& nominal) {
    using Scalar = typename std::decay_t<DeltaVector>::Scalar;
    using Vec3T = Eigen::Matrix<Scalar, 3, 1>;

    for (int i = 0; i < X_N; ++i) {
        const bool rotation_component = i == idx::C_ROT_X || i == idx::C_ROT_Y || i == idx::C_ROT_Z;
        if (!rotation_component)
            nominal[i] += delta[i];
    }

    const Vec3T delta_rot(delta[idx::C_ROT_X], delta[idx::C_ROT_Y], delta[idx::C_ROT_Z]);
    const Vec3T injected_rot = tools::so3_log((state_rotation(nominal) * tools::so3_exp(delta_rot)).eval());
    nominal[idx::C_ROT_X] = injected_rot.x();
    nominal[idx::C_ROT_Y] = injected_rot.y();
    nominal[idx::C_ROT_Z] = injected_rot.z();
}

template<class StateVector, class DeltaVector>
inline void
box_minus_state(const StateVector& nominal, const StateVector& value, DeltaVector& delta) {
    using Scalar = typename std::decay_t<StateVector>::Scalar;
    using Vec3T = Eigen::Matrix<Scalar, 3, 1>;

    delta = value - nominal;
    const Vec3T delta_rot =
        tools::so3_log((state_rotation(nominal).transpose() * state_rotation(value)).eval());
    delta[idx::C_ROT_X] = delta_rot.x();
    delta[idx::C_ROT_Y] = delta_rot.y();
    delta[idx::C_ROT_Z] = delta_rot.z();
}
struct Voter {  // 根据偏航角变化判断旋转方向
    enum {  // 定义一个匿名枚举类型，用来表示当前状态
        Collecting,        // 状态：正在收集数据/尚未判断出方向
        Clockwise,         // 状态：检测到顺时针旋转
        Counterclockwise,  // 状态：检测到逆时针旋转
    } state = Collecting;  // 声明一个匿名枚举变量 state，默认初始化为 Collecting

    void reset(const TimePoint& t) {  // 重置函数，参数t表示当前时间点
        *this = {};  // 把当前对象的所有成员恢复为默认值（值初始化）
        start_t = t;  // 把起始时间设置为传入的时间t
    }

    void update(double yaw, const TimePoint& t) {  // 更新函数，yaw当前偏航角，t是当前时间点
        if (t - start_t < std::chrono::milliseconds(1000)) {  // 如果距离起始时间还不到1000毫秒
            return;  // 直接返回，不做后续判断（相当于启动后1秒内不处理）
        }

        const double diff = angles::normalize_angle(yaw - last_state_yaw);  // 计算当前偏航角与上次记录偏航角的差值，并归一化到[-pi, pi]
        if (std::abs(diff) < 0.05) {  // 如果角度变化太小（小于0.05弧度）
            return;  // 直接返回，认为没有明显转动
        }

        if (diff > 0) {  // 如果角度差为正
            clock_wise_count++;  // 顺时针计数加一
        } else {  // 否则角度差为负
            clock_wise_count--;  // 顺时针计数减一（相当于逆时针计数增加）
        }

        last_state_yaw = yaw;  // 更新上次记录的偏航角为当前偏航角

        if (std::abs(clock_wise_count) > 10) {  // 如果顺时针计数的绝对值超过10
            state = clock_wise_count > 0 ? Clockwise : Counterclockwise;  // 根据计数正负设置状态为顺时针或逆时针
        } else {  // 否则计数还不够大
            state = Collecting;  // 状态保持/恢复为收集状态
        }
    }

    TimePoint start_t;  // 记录起始时间点
    int clock_wise_count = 0;  // 顺时针方向累计计数，初始为0
    double last_state_yaw = 0.0;  // 上一次记录的偏航角，初始为0.0
}; 

struct Predict {
    double dt { 0.0 };

    auto_aim::ArmorType armor_type = auto_aim::ArmorType::UNKNOWN;
    Voter voter;
    template<typename T>
    inline void operator()(const T x0[X_N], T x1[X_N]) const {
        std::copy(x0, x0 + X_N, x1);

        x1[idx::CX] += x0[idx::VCX] * T(dt);
        x1[idx::CY] += x0[idx::VCY] * T(dt);
        x1[idx::CZ] += x0[idx::VCZ] * T(dt);

        if (armor_type != auto_aim::ArmorType::BASE) {
            Eigen::Matrix<T, 3, 1> delta_rot;
            if (armor_type == ArmorType::OUTPOST) {
                if (voter.state == Voter::Clockwise) {
                    delta_rot << T(0), T(0), T(OUTPOST_WZ) * T(dt);
                    x1[idx::VYAW] = T(OUTPOST_WZ);
                } else if (voter.state == Voter::Counterclockwise) {
                    delta_rot << T(0), T(0), -T(OUTPOST_WZ) * T(dt);
                    x1[idx::VYAW] = -T(OUTPOST_WZ);
                } else {
                    delta_rot << T(0), T(0), x0[idx::VYAW] * T(dt);
                }
            } else {
                delta_rot << T(0), T(0), x0[idx::VYAW] * T(dt);
            }

            const Eigen::Matrix<T, 3, 3> R1 =
                (tools::so3_exp(
                     Eigen::Matrix<T, 3, 1>(x0[idx::C_ROT_X], x0[idx::C_ROT_Y], x0[idx::C_ROT_Z])
                 )
                 * tools::so3_exp(delta_rot))
                    .eval();
            auto rot_vec = tools::so3_log(R1);
            x1[idx::C_ROT_X] = rot_vec.x();
            x1[idx::C_ROT_Y] = rot_vec.y();
            x1[idx::C_ROT_Z] = rot_vec.z();
        }

        clamp(x1);
    }

    template<typename T>
    inline void clamp(T x[X_N]) const {
        auto& h = x[idx::H];
        auto& vyaw = x[idx::VYAW];
        x[idx::LOG_R1] = ceres::fmax(
            T(std::log(MIN_ARMOR_R)),
            ceres::fmin(T(std::log(MAX_ARMOR_R)), x[idx::LOG_R1])
        );
        if (armor_type != auto_aim::ArmorType::OUTPOST) {
            x[idx::LOG_R2] = ceres::fmax(
                T(std::log(MIN_ARMOR_R)),
                ceres::fmin(T(std::log(MAX_ARMOR_R)), x[idx::LOG_R2])
            );

            if (ceres::abs(h) > T(0.5)) {
                h = T(0.0);
            }
        } else {
            if (ceres::abs(x[idx::OUTPOST01DZ]) > T(0.3)) {
                x[idx::OUTPOST01DZ] = T(0.0);
            }
            if (ceres::abs(x[idx::OUTPOST02DZ]) > T(0.3)) {
                x[idx::OUTPOST02DZ] = T(0.0);
            }
            x[idx::LOG_R1] = T(std::log(OUTPOST_R));
        }

        if (ceres::abs(vyaw) > T(20.0)) {
            vyaw = T(0.0);
        }
        if (armor_type == auto_aim::ArmorType::BASE) {
            x[idx::VYAW] = T(0.0);
        }
    }
    inline void f(const VecX& x0, VecX& x1) const {
        assert(x0.size() == X_N);
        assert(x1.size() == X_N);
        operator()(x0.data(), x1.data());
    }
};
struct UVCtx {
    int armor_num;
    int id;
    ISO3 T_camera_cv2world = ISO3::Identity();
    CameraInfo camera_info;
    auto_aim::ArmorType armor_type = auto_aim::ArmorType::UNKNOWN;
    bool is_left;
    bool normalized = false;
};
struct UVLMeasure {
    template<typename T>
    using ImagePoint = Eigen::Matrix<T, 2, 1>;

    UVCtx ctx;

    template<typename T>
    inline std::array<ImagePoint<T>, 2> project_points(const T x[X_N]) const {
        auto T_armor2world = armor_pose(x, ctx.id, ctx.armor_num, ctx.armor_type);

        Eigen::Transform<T, 3, Eigen::Isometry> T_camera_cv2world_jet;
        T_camera_cv2world_jet.matrix() = ctx.T_camera_cv2world.matrix().template cast<T>();

        auto T_armor2camera_cv = T_camera_cv2world_jet.inverse() * T_armor2world;

        std::vector<cv::Point3f> object_points =
            getArmorLightKeyPoints3D<cv::Point3f>(ctx.armor_type, ctx.is_left);

        std::vector<ImagePoint<T>> pts_jet;
        if (ctx.normalized) {
            tools::project_points_jets_normalized(
                object_points,
                T_armor2camera_cv,
                ctx.camera_info.distortion_coefficients,
                pts_jet
            );
        } else {
            tools::project_points_jets(
                object_points,
                T_armor2camera_cv,
                ctx.camera_info.camera_matrix,
                ctx.camera_info.distortion_coefficients,
                pts_jet
            );
        }

        return { pts_jet[0], pts_jet[1] };
    }

    template<typename T>
    static inline void
    points_to_observation(const ImagePoint<T>& top, const ImagePoint<T>& bottom, T z[UVLZ_N]) {
        const ImagePoint<T> delta = top - bottom;
        const ImagePoint<T> center = (top + bottom) / T(2);
        z[idx::UV_ANGLE] = ceres::atan2(delta.x(), delta.y());
        z[idx::UV_CENTER_X] = center.x();
        z[idx::UV_CENTER_Y] = center.y();
        z[idx::UV_LENGTH] = ceres::sqrt(delta.squaredNorm());
    }

    template<typename T>
    inline void operator()(const T x[X_N], T z[UVLZ_N]) const {
        const auto points = project_points(x);
        points_to_observation(points[0], points[1], z);
    }

    inline std::pair<cv::Point2f, cv::Point2f> projected_points(const VecX& x) const {
        const auto points = project_points(x.data());
        return { cv::Point2f(points[0].x(), points[0].y()),
                 cv::Point2f(points[1].x(), points[1].y()) };
    }

    template<typename T>
    static inline Eigen::Matrix<T, UVLZ_N, 1>
    residual(const Eigen::Matrix<T, UVLZ_N, 1>& z_pred, const Eigen::Matrix<T, UVLZ_N, 1>& z) {
        Eigen::Matrix<T, UVLZ_N, 1> v = z - z_pred;
        v[idx::UV_ANGLE] = normalize_angle(v[idx::UV_ANGLE]);
        return v;
    }
    inline void h(const VecX& x, UVLVecZ& z) const {
        operator()(x.data(), z.data());
    }
};
struct DiffMeasure {
    template<typename T>
    using ImagePoint = Eigen::Matrix<T, 2, 1>;

    UVCtx ctx;

    template<typename T>
    inline Eigen::Transform<T, 3, Eigen::Isometry> armor_pose_in_camera_cv(const T x[X_N]) const {
        auto pose_in_odom = armor_pose(x, ctx.id, ctx.armor_num, ctx.armor_type);

        Eigen::Transform<T, 3, Eigen::Isometry> camera_cv_in_odom_jet;
        camera_cv_in_odom_jet.matrix() = ctx.T_camera_cv2world.matrix().template cast<T>();

        return camera_cv_in_odom_jet.inverse() * pose_in_odom;
    }

    template<typename T>
    inline void operator()(const T x[X_N], T z[DIFFZ_N]) const {
        const auto pose_in_camera_cv = armor_pose_in_camera_cv(x);
        const auto object_points = getArmorKeyPoints3D<cv::Point3f>(ctx.armor_type);
        auto point_in_camera = [&](ArmorKeyPointsIndex index) {
            const auto& p = object_points[std::to_underlying(index)];
            return pose_in_camera_cv * Eigen::Matrix<T, 3, 1>(T(p.x), T(p.y), T(p.z));
        };
        const auto left_center = (point_in_camera(ArmorKeyPointsIndex::LEFT_TOP)
                                  + point_in_camera(ArmorKeyPointsIndex::LEFT_BOTTOM))
            * T(0.5);
        const auto right_center = (point_in_camera(ArmorKeyPointsIndex::RIGHT_TOP)
                                   + point_in_camera(ArmorKeyPointsIndex::RIGHT_BOTTOM))
            * T(0.5);
        // const auto top_center = (point_in_camera(ArmorKeyPointsIndex::LEFT_TOP)
        //                           + point_in_camera(ArmorKeyPointsIndex::RIGHT_TOP))
        //     * T(0.5);
        // const auto bottom_center = (point_in_camera(ArmorKeyPointsIndex::LEFT_BOTTOM)
        //                              + point_in_camera(ArmorKeyPointsIndex::RIGHT_BOTTOM))
        //     * T(0.5);
        z[0] = left_center.z() - right_center.z();
        // z[1] = top_center.z() - bottom_center.z();
    }

    template<typename T>
    static inline Eigen::Matrix<T, DIFFZ_N, 1>
    residual(const Eigen::Matrix<T, DIFFZ_N, 1>& z_pred, const Eigen::Matrix<T, DIFFZ_N, 1>& z) {
        Eigen::Matrix<T, DIFFZ_N, 1> v = z - z_pred;
        return v;
    }
    inline void h(const VecX& x, DiffVecZ& z) const {
        operator()(x.data(), z.data());
    }
};

struct YPDMeasure {
    struct Ctx {
        int armor_num;
        int id;
        auto_aim::ArmorType armor_type = auto_aim::ArmorType::UNKNOWN;
        ISO3 T_camera_cv2world = ISO3::Identity();
    } ctx;

    template<typename T>
    inline void operator()(const T x[X_N], T z[YPDZ_N]) const {
        auto T_armor2world = armor_pose(x, ctx.id, ctx.armor_num, ctx.armor_type);
        Eigen::Transform<T, 3, Eigen::Isometry> T_camera_cv2world_jet;
        T_camera_cv2world_jet.matrix() = ctx.T_camera_cv2world.matrix().template cast<T>();
        auto T_armor2camera_cv = T_camera_cv2world_jet.inverse() * T_armor2world;
        T armor_x = T_armor2camera_cv.translation().x();
        T armor_y = T_armor2camera_cv.translation().y();
        T armor_z = T_armor2camera_cv.translation().z();
        auto rot_vec = tools::so3_log<T>(T_armor2camera_cv.linear());
        T xy_dist = ceres::sqrt(armor_x * armor_x + armor_y * armor_y);
        T dist = ceres::sqrt(xy_dist * xy_dist + armor_z * armor_z);
        // Observation model
        z[idx::YPD_Y] = ceres::atan2(armor_y, armor_x); // yaw
        z[idx::YPD_P] = ceres::atan2(armor_z, xy_dist); // pitch
        z[idx::YPD_D] = dist; // distance
        // z[idx::A_ROT_YAW] = ypr[0];
        z[idx::A_ROT_X] = rot_vec.x();
        z[idx::A_ROT_Y] = rot_vec.y();
        z[idx::A_ROT_Z] = rot_vec.z();
    }
    template<typename T>
    static inline Eigen::Matrix<T, YPDZ_N, 1>
    residual(const Eigen::Matrix<T, YPDZ_N, 1>& z_pred, const Eigen::Matrix<T, YPDZ_N, 1>& z) {
        Eigen::Matrix<T, YPDZ_N, 1> v = z - z_pred;
        v[idx::YPD_Y] = normalize_angle(v[idx::YPD_Y]);
        // v[idx::A_ROT_YAW] = normalize_angle(v[idx::A_ROT_YAW]);
        auto so3_residual = tools::so3_log(
            (tools::so3_exp(Vec3(z[idx::A_ROT_X], z[idx::A_ROT_Y], z[idx::A_ROT_Z]))
             * tools::so3_exp(Vec3(z_pred[idx::A_ROT_X], z_pred[idx::A_ROT_Y], z_pred[idx::A_ROT_Z])
             )
                   .transpose())
                .eval()
        );
        v[idx::A_ROT_X] = so3_residual.x();
        v[idx::A_ROT_Y] = so3_residual.y();
        v[idx::A_ROT_Z] = so3_residual.z();
        return v;
    }
    inline void h(const VecX& x, YPDVecZ& z) const {
        operator()(x.data(), z.data());
    }
};

struct State {
    VecX x;
    TimePoint timestamp;
    int frame_id = 0;

    inline std::vector<ISO3> get_armors_pose(auto_aim::ArmorType armor_type) const {
        std::vector<ISO3> T_armors2world;
        int armor_num = armor_num_by_armor_type(armor_type);
        T_armors2world.reserve(armor_num);
        for (int i = 0; i < armor_num; ++i) {
            auto T_armor2world = armor_pose(x.data(), i, armor_num, armor_type);
            T_armors2world.push_back(T_armor2world);
        }

        return T_armors2world;
    }

    inline void predict(const TimePoint& t, auto_aim::ArmorType armor_type) {
        auto dt = std::chrono::duration<double>(t - timestamp).count();
        predict(dt, armor_type);
    }
    inline void predict(double dt, auto_aim::ArmorType armor_type) {
        Predict p { .dt = dt, .armor_type = armor_type };
        auto tmp_x = x;
        p.f(tmp_x, x);
        timestamp +=
            std::chrono::duration_cast<TimePoint::duration>(std::chrono::duration<double>(dt));
    }
    inline double get_armor_r(int id, auto_aim::ArmorType armor_type) const {
        return armor_radius(x.data(), id, armor_num_by_armor_type(armor_type), armor_type);
    }
    inline void set_pos(const Vec3& p) noexcept {
        x[idx::CX] = p.x();
        x[idx::CY] = p.y();
        x[idx::CZ] = p.z();
    }
    inline Vec3 pos() const noexcept {
        return Vec3(x[idx::CX], x[idx::CY], x[idx::CZ]);
    }
    inline void set_vel(const Vec3& v) noexcept {
        x[idx::VCX] = v.x();
        x[idx::VCY] = v.y();
        x[idx::VCZ] = v.z();
    }
    inline Vec3 vel() const noexcept {
        return Vec3(x[idx::VCX], x[idx::VCY], x[idx::VCZ]);
    }

    inline Vec3 ypr() const noexcept {
        return tools::matrix2euler(
            tools::so3_exp(Vec3(x[idx::C_ROT_X], x[idx::C_ROT_Y], x[idx::C_ROT_Z])),
            tools::EulerOrder::ZYX
        );
    }
    inline double yaw() const noexcept {
        return ypr().x();
    }
    inline double vyaw() const noexcept {
        return x[idx::VYAW];
    }

    inline double r1() const noexcept {
        return ceres::exp(x[idx::LOG_R1]);
    }
    inline double r2() const noexcept {
        return ceres::exp(x[idx::LOG_R2]);
    }
    inline double h() const noexcept {
        return x[idx::H];
    }
    inline double outpost01DZ() const noexcept {
        return x[idx::OUTPOST01DZ];
    }
    inline double outpost02DZ() const noexcept {
        return x[idx::OUTPOST02DZ];
    }
    inline double w_p() const noexcept {
        return ypr().y();// pitch
    }
    inline double w_r() const noexcept {
        return ypr().z();// roll
    }
};

using RobotStateESEKF = kalman_hybird_lib::ErrorStateEKF<X_N, Predict>;

} // namespace dts_vision::auto_aim::armor_point_motion_model
