#pragma once
#include "tools/common/type_common.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <shared_mutex>

namespace dts_vision::tools {

struct TimedPose {// 时间戳+位姿
    TimePoint stamp;
    ISO3 pose;
};

class TimePoseBuffer {
public:
    explicit TimePoseBuffer(size_t max_size = 1024): max_size_(max_size) {}// 构造函数，禁止隐式转换

    void push(const TimePoint& t, const ISO3& pose) {
        std::unique_lock<std::shared_mutex> lock(mutex_);// 写入时加锁
        if (max_size_ == 0)
            return;
        if (!buffer_.empty() && t < buffer_.back().stamp) {
            std::cout << "Time is out of order!" << std::endl;
            return;
        }// 时间戳必须按顺序插入

        buffer_.push_back({ t, pose });
        while (buffer_.size() > max_size_)
            buffer_.pop_front();// 超出最大值时，删除最前面的
    }

    ISO3 get(const TimePoint& t) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);

        if (buffer_.empty())
            return ISO3::Identity();
        if (buffer_.size() == 1)
            return buffer_.front().pose;
        // 在一个按时间戳排好序的容器(buffer_)里，使用二分查找快速找到第一个时间戳大于或等于t的元素
        // 如果没有大于或等于t的元素，std::lower_bound会返回buffer_.end()(尾后迭代器。它不指向容器里的任何元素，而是指向最后一个元素之后的假想位置)
        auto it = std::lower_bound(
            buffer_.begin(),
            buffer_.end(),
            t,
            [](const TimedPose& p, const TimePoint& t) { return p.stamp < t; }
        );

        if (it == buffer_.begin()) {
            // std::cout << "its begin" << std::endl;
            return it->pose;
        }
        // 
        if (it == buffer_.end()) {// 处理t比buffer_里所有元素的时间戳都要晚的情况
            // std::cout << "its end" << std::endl;
            return extrapolate(t);
        }

        const auto& p1 = *(it - 1);
        const auto& p2 = *it;

        const double dt = seconds(p2.stamp - p1.stamp);
        if (std::abs(dt) < kMinDt)
            return p2.pose;

        double r = std::clamp(seconds(t - p1.stamp) / dt, 0.0, 1.0);

        Vec3 trans = (1 - r) * p1.pose.translation() + r * p2.pose.translation();
        Quaternion q1(p1.pose.rotation());
        Quaternion q2(p2.pose.rotation());
        normalize_pair(q1, q2);
        Quaternion q = q1.slerp(r, q2).normalized();

        ISO3 T = ISO3::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = trans;
        return T;
    }

private:
    static constexpr double kMinDt = 1e-6;
    static constexpr double kMinOmega = 1e-9;

    template<typename Duration>
    static double seconds(const Duration& duration) {
        return std::chrono::duration<double>(duration).count();// 将时间间隔转换为秒
    }

    static void normalize_pair(Quaternion& q1, Quaternion& q2) {
        q1.normalize();
        q2.normalize();
        if (q1.dot(q2) < 0.0)
            q2.coeffs() *= -1.0;
    }

    ISO3 extrapolate(const TimePoint& t) const {
        if (buffer_.size() < 2)
            return buffer_.back().pose;

        const auto& p1 = buffer_[buffer_.size() - 2];
        const auto& p2 = buffer_.back();

        double dt = seconds(p2.stamp - p1.stamp);
        if (dt < kMinDt)
            return p2.pose;// dt太小，没必要沿着轨迹外推

        double dt_future = seconds(t - p2.stamp);

        Vec3 v = (p2.pose.translation() - p1.pose.translation()) / dt;// [v_x, v_y, v_z]
        Vec3 trans = p2.pose.translation() + v * dt_future;// 当匀速运动推

        Quaternion q1(p1.pose.rotation());
        Quaternion q2(p2.pose.rotation());
        normalize_pair(q1, q2);
        Quaternion dq = q1.inverse() * q2;
        dq.normalize();

        AngleAxis aa(dq);
        Vec3 omega = aa.axis() * aa.angle() / dt;

        double omega_norm = omega.norm();
        double angle = omega_norm * dt_future;
        Vec3 axis = Vec3::UnitX();
        if (omega_norm > kMinOmega)
            axis = omega / omega_norm;

        Quaternion q_future = (q2 * Quaternion(AngleAxis(angle, axis))).normalized();

        ISO3 T = ISO3::Identity();
        T.linear() = q_future.toRotationMatrix();
        T.translation() = trans;
        return T;
    }

private:
    mutable std::shared_mutex mutex_;
    std::deque<TimedPose> buffer_;
    size_t max_size_;
};

} // namespace dts_vision::tools
