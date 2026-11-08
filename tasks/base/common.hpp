#pragma once
#include "tools/common/image.hpp"
#include "tools/common/type_common.hpp"
#include "tools/utils.hpp"
namespace dts_vision {
static const Mat3 R_cv2physics =
    (Mat3() << 0.0, 0.0, 1.0, -1.0, -0.0, 0.0, 0.0, -1.0, 0.0).finished();
struct CommonFrame {
    ImageFrame img_frame;
    int id;
    int frame_id;
};
enum class EnemyColor : bool {
    RED = 0,
    BLUE = 1,
};
inline EnemyColor enemy_color_from_string(const std::string& str) {
    auto key = tools::to_upper(str);
    if (key == "RED")
        return EnemyColor::RED;
    else if (key == "BLUE") {
        return EnemyColor::BLUE;
    } else {
        throw std::runtime_error("Invalid enemy color: " + key);
    }
}
struct CameraInfo {
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    void load(const YAML::Node& config) {
        std::vector<double> camera_k = config["camera_matrix"]["data"].as<std::vector<double>>();
        std::vector<double> camera_d =
            config["distortion_coefficients"]["data"].as<std::vector<double>>();

        assert(camera_k.size() == 9);
        assert(camera_d.size() == 5);

        cv::Mat K(3, 3, CV_64F);
        std::memcpy(K.data, camera_k.data(), 9 * sizeof(double));

        cv::Mat D(1, 5, CV_64F);
        std::memcpy(D.data, camera_d.data(), 5 * sizeof(double));

        camera_matrix = K.clone();
        distortion_coefficients = D.clone();
    }
    double camera_fx() const noexcept {
        return camera_matrix.at<double>(0, 0);
    }

    double camera_fy() const noexcept {
        return camera_matrix.at<double>(1, 1);
    }

    double camera_cx() const noexcept {
        return camera_matrix.at<double>(0, 2);
    }

    double camera_cy() const noexcept {
        return camera_matrix.at<double>(1, 2);
    }

    CameraInfo clone() const {
        return { .camera_matrix = this->camera_matrix.clone(),
                 .distortion_coefficients = this->distortion_coefficients.clone() };
    }
};
struct AimPoint {
    ISO3 pose;
    int frame_id;
    static AimPoint lerp(const AimPoint& a, const AimPoint& b, double k) {// 球面线性插值
        AimPoint aim_point;
        // a+(b-a)*k = a*(1-k)+b*k--->起点为a,终点为b,求k/1处位姿
        Vec3 trans = (1.0 - k) * a.pose.translation() + k * b.pose.translation();// 平移部分线性插值
        Quaternion q_a(a.pose.rotation());// a点旋转部分球面线性插值
        Quaternion q_b(b.pose.rotation());
        Quaternion q = q_a.slerp(k, q_b);

        aim_point.pose = ISO3::Identity();
        aim_point.pose.linear() = q.toRotationMatrix();
        aim_point.pose.translation() = trans;
        return aim_point;
    }
    void transform(const ISO3& T_old2new, int new_frame_id) {
        pose = T_old2new * pose;
        frame_id = new_frame_id;
    }
};
struct GimbalCmd {
    TimePoint timestamp;
    double pitch = 0;
    double yaw = 0;
    double enable_yaw_diff = 0;
    double enable_pitch_diff = 0;
    double target_yaw = 0;
    double target_pitch = 0;
    double v_yaw = 0;
    double v_pitch = 0;
    double a_yaw = 0;
    double a_pitch = 0;
    bool fire_advice = false;
    double fly_time = 0;
    bool appear = false;
    AimPoint aim_point;
    int select_id = 0;
    inline bool is_valid() const noexcept {
        auto bad = [](double x) { return std::isnan(x) || std::isinf(x); };

        if (bad(pitch) || bad(yaw) || bad(target_yaw) || bad(target_pitch)
            || bad(v_yaw) || bad(v_pitch) || bad(a_yaw) || bad(a_pitch)
            || bad(enable_yaw_diff) || bad(enable_pitch_diff))
            return false;

        return true;
    }
    inline void no_shoot() {
        fire_advice = false;
        enable_yaw_diff = 0;
        enable_pitch_diff = 0;
    }
};
} // namespace dts_vision
