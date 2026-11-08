#pragma once 
#include "traj.hpp"
#include "tools/common/type_common.hpp" 
#include "tools/utils.hpp" 
#include <deque> 
#include <optional> 
#include <utility>
#include <vector> 
#include <yaml-cpp/node/node.h> 

namespace dts_vision { 

class BallisticTrajectory { // 弹道解算类
public: 
    using Ptr = std::shared_ptr<BallisticTrajectory>; 

    enum class ModelType { // 定义弹道模型类型
        ResistanceApprox,    // 近似阻力模型：水平距离受阻力影响，竖直高度仍按普通抛物线算
        ResistanceApproxFull // 完整线性阻力模型：水平和竖直两个方向都按线性阻力公式算
    }; 

    struct Params { 
        double gravity = 9.8;
        double resistance = 0.092; // 近似阻力模型阻力系数
        double resistance_full = 0.092; // 完整线性阻力模型阻力系数
        std::string compenstator_type = "resistance"; // 配置选择的模型名，默认使用近似阻力模型resistance

        void load(const YAML::Node& config) {
            if (config["gravity"])
                gravity = config["gravity"].as<double>(); 

            if (config["resistance"]) // 如果有resistance，就用配置里的近似阻力系数
                resistance = config["resistance"].as<double>(); 

            if (config["resistance_full"]) 
                resistance_full = config["resistance_full"].as<double>(); 

            if (config["compenstator_type"]) // 如果有compenstator_type，就按这个字段选择模型
                compenstator_type = config["compenstator_type"].as<std::string>(); 
            else if (config["resistance_full"]) // 如果没写模型名但写了resistance_full
                compenstator_type = "resistance_full"; // 自动把模型设成resistance_full

            if (gravity <= 0.0) { 
                throw std::invalid_argument("gravity must be positive"); 
            } 
            if (resistance <= 0.0) { 
                resistance = kEps; // 把它修正成一个很小的正数，避免除零
            } 
            if (resistance_full <= 0.0) { 
                resistance_full = kEps;
            } 
        }

    private: 
        static constexpr double kEps = 1e-6; 
    };

public:
    explicit BallisticTrajectory(const YAML::Node& config) { // 构造弹道解算器，explicit防止隐式类型转换
        params_.load(config); 

        g_ = params_.gravity; 
        k_ = std::max(params_.resistance, kEps); // 近似阻力系数至少取kEps，避免为0
        inv_k_ = 1.0 / k_; // 预先算好1/k，多次用到，避免重复除法
        resistance_full_ = std::max(params_.resistance_full, kEps); 

        if (params_.compenstator_type == "resistance_full") { 
            model_type_ = ModelType::ResistanceApproxFull; 
        } else { 
            model_type_ = ModelType::ResistanceApprox; 
        } // 模型选择
    } // 构造函数

    static Ptr create(const YAML::Node& config) { // 工厂函数：外部用它创建一个智能指针管理的弹道解算器
        return std::make_shared<BallisticTrajectory>(config); // 根据YAML里的弹道配置，创建一个弹道解算器对象，然后用智能指针返回
    } // 工厂函数

    inline std::optional<double> solve_pitch(const Vec3& target_pos, double v0) const { // 根据目标xyz和子弹初速，求需要抬起的pitch角
        if (model_type_ == ModelType::ResistanceApproxFull) { 
            return solve_pitch_resistance_full(target_pos, v0);// BallisticTrajectory的私有函数
        } 
        return solve_pitch_resistance(target_pos, v0);
    } 

    inline std::optional<std::pair<double, double>> // 返回值可能为空,不为空时pair.first--->pitch，pair.second--->飞行时间
    solve_pitch_and_flytime(const Vec3& target_pos, double v0) const { // 同时求枪口pitch和子弹飞到目标水平距离所需的时间
        const double distance = std::hypot(target_pos.x(), target_pos.y()); // sqrt(x^2+y^2)

        auto pitch_opt = solve_pitch(target_pos, v0); // 先调用上面的solve_pitch求出发射角
        if (!pitch_opt) // 如果pitch_opt为空，说明当前目标和速度条件下算不出合法发射角
            return std::nullopt; // 直接返回空，表示求解失败

        double t = 0.0; 
        if (model_type_ == ModelType::ResistanceApproxFull) {
            t = flight_time_resistance_full(distance, v0, *pitch_opt); 
        } else { 
            t = flight_time_resistance(distance, v0, *pitch_opt); 
        }

        if (!std::isfinite(t) || t < 0.0) { 
            return std::nullopt; 
        } 

        return std::make_pair(*pitch_opt, t); 
    } 

    inline std::pair<double, double> solve_distance_height(double pitch, double v0, double t) const { // 给定pitch、初速和飞行时间，反过来算子弹飞了多远、升降了多少
        if (model_type_ == ModelType::ResistanceApproxFull) { 
            return forward_model_resistance_full(pitch, v0, t); // 用完整模型正向计算水平距离和高度变化
        } 
        return forward_model_resistance(pitch, v0, t); // 否则用近似阻力模型正向计算
    } 

private: 
    static constexpr double kEps = 1e-6; 

    Params params_; // 保存配置读出来的原始参数
    ModelType model_type_ = ModelType::ResistanceApprox; // 当前实际使用的模型类型，默认是近似阻力模型

    double k_ = 0.092; // 近似阻力模型的阻力系数
    double inv_k_ = 1.0 / 0.092; // k_的倒数
    double resistance_full_ = 0.092; // 完整线性阻力模型的阻力系数
    double g_ = 9.8; // 当前使用的重力加速度
    /*模型：
    阻力加速度 = -kx，其中k是阻力系数，x是距离--->这个x=sqrt(x^2 + y^2)
    水平方向：v_x'(t) = -k*x(t) ---> dv_x/dt = -kx ---> x''=−kx ---> x'' + kx = 0
    即假设水平速度随飞行距离指数衰减： v_x(x) = v0*cosθ*exp(-kx) ---> dx/dt = v0*cosθ*exp(-kx) ---> exp(kx)*dx/dt = v0*cosθ ---> exp(kx)/k = v0*cosθ*t +C ---> 1/k = C ---> x = ln(v0*cosθ*k*t+1)/k
    kx = ln(v0*cosθ*k*t+1) ---> exp(kx)-1 = v0*cosθ*k*t ---> t = (exp(kx)-1)/(v0*cosθ*k) ---> h = v0*sinθ*(exp(kx)-1)/(v0*cosθ*k)-0.5*g*(exp(kx)-1)/(v0*cosθ*k)^2 
    于是变为求解一个二次方程：h = v0*sinθ*(exp(kx)-1)/(v0*cosθ*k)-0.5*g*(exp(kx)-1)/(v0*cosθ*k)^2 
    令e = exp(kx)-1, A = (exp(kx)-1)/k, C = (exp(kx)-1)/v0*k, A = C*v0 B = 0.5*g*(exp(kx)-1)/(v0**k)^2 
    所以h = C*v0*tanθ - B/cosθ^2 ---> 而1/cosθ^2 = 1+tanθ^2
    所以h = C*v0*tanθ - B*(1+tanθ^2) ---> disc = [A*A-4*B(B+h)] ---> tanθ_1 = A+sqrt(disc)/2*B, tanθ_2 = A-sqrt(disc)/2*B */
    inline std::optional<double> solve_pitch_resistance(const Vec3& target_pos, double v0) const { // 近似阻力模型下求pitch，用解析公式直接算
        const double h = target_pos.z(); // 目标相对枪口的高度差
        const double distance = std::hypot(target_pos.x(), target_pos.y()); // 目标相对枪口的水平距离

        if (distance < kEps || v0 < kEps) { // 水平距离太小或子弹速度太小都会让弹道公式不稳定
            return std::nullopt; // 返回空，表示没有可靠解
        } 

        const double kd = k_ * distance; // 阻力系数乘水平距离，是指数公式里的参数
        if (kd > 700.0) { // kd太大会让exp(kd)溢出，double大约到这个量级就很危险
            return std::nullopt; // 返回空，表示目标太远或阻力太大，当前公式不能稳定计算
        }

        const double e = std::expm1(kd); // 计算exp(kd)-1，当x非常接近0时，直接计算exp(x)-1会损失精度(两个接近1的数相减会丢失有效位），而expm1用数值稳定的算法直接给出准确结果
        const double A = e * inv_k_; // A相当于把受阻力影响的水平运动整理成一个中间量
        const double C = A / v0; // C是A除以初速，后面用来构造高度方程
        const double B = 0.5 * g_ * C * C; // B是重力项整理后的系数

        const double disc = A * A - 4.0 * B * (B + h); // 二次方程判别式，决定tan(pitch)有没有实数解
        if (disc < 0.0 || !std::isfinite(disc)) { // 判别式小于0表示无实数解，非有限值表示数值已经坏掉
            return std::nullopt; // 无法得到合法pitch，返回空
        } 

        const double sqrt_disc = std::sqrt(disc); // 对判别式开方，用来求二次方程的两个根
        const double denom = 2.0 * B; // 二次方程求根公式的分母
        if (std::abs(denom) < kEps) { // 分母太小近似除0
            return std::nullopt; // 返回空，避免得到不可信的角度
        } 

        const double u1 = (A + sqrt_disc) / denom; // tan(pitch)的第一个候选解，通常对应高抛角
        const double u2 = (A - sqrt_disc) / denom; // tan(pitch)的第二个候选解，通常对应低抛角

        const double p1 = std::atan(u1); // 把第一个tan值转成pitch角，rad
        const double p2 = std::atan(u2); // 把第二个tan值转成pitch角

        constexpr double left = -M_PI / 4.0; // 允许的最小pitch，-45°
        constexpr double right = M_PI / 3.0; // 允许的最大pitch，60°

        auto valid = [&](double pitch) { // 定义一个lambda函数，用来判断候选pitch是否能用
            return std::isfinite(pitch) && pitch >= left && pitch <= right && std::abs(std::cos(pitch)) > kEps; // 必须是有限值、在角度范围内，并且cos不能太接近0
        }; 

        if (valid(p1)) // 如果第一个候选角合法
            return p1; // 返回第一个候选角
        if (valid(p2)) // 如果第一个不合法但第二个合法
            return p2; // 返回第二个候选角

        return std::nullopt; // 两个候选角都不能用，返回空
    } 
    //  t = (exp(kx)-1)/(v0*cosθ*k) 
    inline double flight_time_resistance(double distance, double v0, double pitch) const { // 近似阻力模型下，根据水平距离、初速、pitch算飞行时间
        const double cos_theta = std::cos(pitch); // 计算cos(pitch)，水平速度等于 v0*cos(pitch)
        if (std::abs(cos_theta) < kEps) { // cosθ太接近0表示枪口接近竖直，水平速度几乎为0
            return std::numeric_limits<double>::infinity(); // 返回无穷大，表示飞不到这个水平距离
        } 

        const double kd = k_ * distance; // 指数公式中的k*距离
        if (kd > 700.0) { // kd过大会造成exp溢出
            return std::numeric_limits<double>::infinity(); // 返回无穷大，表示飞行时间不可用
        } 
        return std::expm1(kd) / (k_ * v0 * cos_theta); // 近似阻力模型的飞行时间公式
    } 
    // x = ln(v0*cosθ*k*t+1)/k
    // y = v0*sinθ-0.5*g*t^2
    inline std::pair<double, double> // 返回pair.first为水平距离，pair.second为高度变化
    forward_model_resistance(double pitch, double v0, double t) const { // 近似阻力模型正向计算：已知发射角和时间，算子弹位置变化
        const double cos_theta = std::cos(pitch); // 计算cos(pitch)，用来得到水平速度分量
        const double x = std::log1p(k_ * v0 * cos_theta * t) * inv_k_; // 根据近似阻力公式计算t秒后的水平距离
        const double y = v0 * std::sin(pitch) * t - 0.5 * g_ * t * t; // 竖直方向按无阻力抛物线计算高度变化

        return { x, y }; // 返回水平距离x和高度变化y
    } 
    /*模型：
    阻力加速度 = -kv
    水平方向：v_x'(t) = -k*v_x(t) ---> 1/v_x * dv_x/dt = -k ---> ln(v_x) = -kt + C ---> ln(v_x(0)) = C ---> v_x(t) = v_x(0)*exp(-kt) = v0*cosθ*exp(-kt)
    积分得x(t) = C-v0*cosθ*exp(-kt)/k ---> x(0) = 0 = C - v0*cosθ/k ---> C = v0*cosθ ---> x(t) = v0*cosθ*(1-exp(-kt))/k 
    distance = x = v0*cosθ*(1-exp(-kt))/k ---> t = k*distance/(v0*cosθ)(exp(kx)-1)/(v0*cosθ*k)
    竖直方向同时受重力和线性阻力：
    v_z'(t) = -g - k*v_z(t) ---> k*v_z + dv_z/dt = -g ---> 构造k*exp(kt)*v_z + exp(kt)dv_z/dt = -g*exp(kt) ---> 积分得exp(kt)*v_z(t) = -g*exp(kt)/k + C ---> v_z(0) = v0*sinθ = -g/k + C ---> C = v0*sinθ + g/k ---> exp(kt)*v_z(t) = g*(1-exp(kt))/k + v0*sinθ ---> v_z(t) = g*(exp(-kt)-1)/k + v0*sinθ*exp(-kt)
    v_z(t) = g*(exp(-kt)-1)/k + v0*sinθ*exp(-kt)积分得z(t) = -g*exp(-kt)/k^2 -g*t/k - v0*sinθ*exp(-kt)/k + C ---> z(0) = 0 = -g/k^2 - v0*sinθ/k + C ---> C = g/k^2 + v0*sinθ/k ---> z(t) = -g*exp(-kt)/k^2 - g*t/k - v0*sinθ*exp(-kt)/k + g/k^2 + v0*sinθ/k ---> z(t) = v0*sinθ*t(1-exp(-kt))/k + g*((1-exp(-kt)))/k^2 - g*t/k*/
    inline std::optional<double> solve_pitch_resistance_full(const Vec3& target_pos, double v0) const { // 完整线性阻力模型下求pitch，这里没有简单解析解，所以用扫描加二分
        const double distance = std::hypot(target_pos.x(), target_pos.y()); // 目标水平距离
        const double h = target_pos.z(); // 目标高度

        if (distance < kEps || v0 < kEps) { // 水平距离太小或初速度太小，公式不好稳定求解
            return std::nullopt; // 返回空，表示无法求pitch
        } 

        auto height_error = [&](double pitch) -> double { // 给一个pitch，算子弹到目标水平距离时的高度误差
            const double cos_theta = std::cos(pitch); // 水平速度比例
            const double sin_theta = std::sin(pitch); // 竖直速度比例

            const double denom = v0 * cos_theta; // 初始水平速度
            if (denom <= kEps) { // 水平速度太小或方向不对，无法飞到目标水平距离
                return std::numeric_limits<double>::infinity(); // 返回无穷大误差，表示这个pitch不可用
            } 
            // t = k*distance/(v0*cosθ)(exp(kx)-1)/(v0*cosθ*k)
            const double ratio = resistance_full_ * distance / denom; // 完整阻力模型里求时间需要的比例项
            if (ratio >= 1.0 || ratio < 0.0) { // ratio必须在[0,1)内，否则log1p(-ratio)会溢出--->log1p(-ratio)=ln(1-ratio)
                return std::numeric_limits<double>::infinity(); // 返回无穷大误差，表示这个角度不可用
            } 

            const double t = -std::log1p(-ratio) / resistance_full_; // std::log1p(-ratio)：调用标准库函数std::log1p(x)，它计算ln⁡(1+x)，且x接近0时比std::log(1+x)更精确,这里x=-ratio，所以计算ln⁡(1−ratio)
            if (!std::isfinite(t)) { // 如果时间是NaN或无穷大
                return std::numeric_limits<double>::infinity(); // 返回无穷大误差，表示这个角度不可用
            } 

            const double decay = -std::expm1(-resistance_full_ * t); // 计算1-exp(-k*t)，用expm1提高小数值精度
            const double z_pred = ((v0 * sin_theta + g_ / resistance_full_) / resistance_full_) * decay - (g_ / resistance_full_) * t; // 完整阻力模型下t秒后的预测高度

            return z_pred - h; // 子弹下降高度减目标高度,=0时说明这个pitch正好命中高度
        }; 

        constexpr double left_bound = -0.7853; // pitch搜索左边界，约等于-45度
        constexpr double right_bound = 0.7853; // pitch搜索右边界，约等于45度
        constexpr int samples = 180; // 先把搜索范围切成180份，用来找误差变号区间

        double last_pitch = left_bound; // 保存上一个采样点的pitch
        double last_h_err = height_error(last_pitch); // 保存上一个采样点的高度误差

        for (int i = 1; i <= samples; ++i) { // 从左到右扫描所有采样点，寻找误差从正到负或从负到正的区间
            const double cur_pitch = left_bound + (right_bound - left_bound) * i / samples; // 当前采样点对应的pitch
            const double cur_h_err = height_error(cur_pitch); // 当前pitch对应的高度误差

            if (std::isfinite(last_h_err) && std::isfinite(cur_h_err) && last_h_err * cur_h_err <= 0.0) { // 如果相邻两个点误差异号，说明在上一次pitch和本次pitch间至少有一个解
                double left = last_pitch; // 二分搜索区间左端
                double right = cur_pitch; // 二分搜索区间右端
                double left_h_err = last_h_err; // 左端点的误差值

                for (int it = 0; it < 60; ++it) { // 在找到的区间里做60次二分，精度已经足够高
                    const double mid = 0.5 * (left + right); // 当前区间中点
                    const double mid_h_err = height_error(mid); // 中点pitch对应的高度误差

                    if (!std::isfinite(mid_h_err)) { // 如果中点算出来无效，可能是ratio = resistance_full_ * distance / denom，log1p(-ratio)=ln(1-ratio)，这里pitch过大，cos(pitch)过小，导致ratio>1,右侧可能靠近非法区域
                        right = mid; // 把右端收缩到中点
                        continue; // 跳过本轮后续判断，继续下一次二分
                    }

                    if (std::abs(mid_h_err) < 1e-6) { // 如果高度误差已经非常接近0
                        return mid; // 直接返回当前中点作为pitch
                    } 

                    if (left_h_err * mid_h_err <= 0.0) { // 如果根还在左端点和中点之间
                        right = mid; // 右端缩到中点
                    } else { // 否则根在中点和原右端之间
                        left = mid; // 左端移动到中点
                        left_h_err = mid_h_err; // 更新左端误差，方便下一轮判断
                    } 
                }
                return 0.5 * (left + right); // 如果60次后还没提前返回，就返回最终区间中点
            }

            last_pitch = cur_pitch; // 当前采样点变成下一轮的上一个采样点
            last_h_err = cur_h_err; // 当前误差变成下一轮的上一个误差
        } 

        return std::nullopt; // 没找到误差变号区间，说明当前范围内没有可用pitch
    }

    inline double flight_time_resistance_full(double distance, double v0, double pitch) const { // 完整线性阻力模型下计算飞行时间
        const double cos_theta = std::cos(pitch); // 初始水平速度
        if (std::abs(cos_theta) < kEps) { // 如果水平速度比例太小
            return std::numeric_limits<double>::infinity(); // 返回无穷大，表示无法飞到指定水平距离
        } 

        const double ratio = resistance_full_ * distance / (v0 * cos_theta);
        if (ratio >= 1.0 || ratio < 0.0) { // ratio超出[0,1)会导致对数公式无效
            return std::numeric_limits<double>::infinity(); // 返回无穷大，表示飞行时间不可用
        } 
        // t = k*distance/(v0*cosθ)(exp(kx)-1)/(v0*cosθ*k)
        return -std::log1p(-ratio) / resistance_full_; // 由完整阻力水平位移公式反解出的飞行时间
    } 

    inline std::pair<double, double> // 返回pair.first为水平距离，pair.second为高度变化
    forward_model_resistance_full(double pitch, double v0, double t) const { // 完整阻力模型正向计算：已知发射角和时间，算子弹位置变化
        const double cos_theta = std::cos(pitch); // 水平初速度分量
        const double sin_theta = std::sin(pitch); // 竖直初速度分量

        const double decay = -std::expm1(-resistance_full_ * t); // 计算1-exp(-k*t)，完整阻力公式里水平和竖直方向都会用到
        /*水平方向：v_x'(t) = -k*v_x(t) ---> 1/v_x * dv_x/dt = -k ---> ln(v_x) = -kt + C ---> ln(v_x(0)) = C ---> v_x(t) = v_x(0)*exp(-kt) = v0*cosθ*exp(-kt)
        积分得x(t) = C-v0*cosθ*exp(-kt)/k ---> x(0) = 0 = C - v0*cosθ/k ---> C = v0*cosθ ---> x(t) = v0*cosθ*(1-exp(-kt))/k 
        distance = x = v0*cosθ*(1-exp(-kt))/k ---> t = k*distance/(v0*cosθ)(exp(kx)-1)/(v0*cosθ*k)
        竖直方向同时受重力和线性阻力：
        v_z'(t) = -g - k*v_z(t) ---> k*v_z + dv_z/dt = -g ---> 构造k*exp(kt)*v_z + exp(kt)dv_z/dt = -g*exp(kt) ---> 积分得exp(kt)*v_z(t) = -g*exp(kt)/k + C ---> v_z(0) = v0*sinθ = -g/k + C ---> C = v0*sinθ + g/k ---> exp(kt)*v_z(t) = g*(1-exp(kt))/k + v0*sinθ ---> v_z(t) = g*(exp(-kt)-1)/k + v0*sinθ*exp(-kt)
        v_z(t) = g*(exp(-kt)-1)/k + v0*sinθ*exp(-kt)积分得z(t) = -g*exp(-kt)/k^2 -g*t/k - v0*sinθ*exp(-kt)/k + C ---> z(0) = 0 = -g/k^2 - v0*sinθ/k + C ---> C = g/k^2 + v0*sinθ/k
        z(t) = -g*exp(-kt)/k^2 - g*t/k - v0*sinθ*exp(-kt)/k + g/k^2 + v0*sinθ/k ---> z(t) = v0*sinθ*t(1-exp(-kt))/k + g*((1-exp(-kt)))/k^2 - g*t/k*/
        const double x = (v0 * cos_theta / resistance_full_) * decay; // 完整线性阻力下t秒后的水平距离变化
        const double y = ((v0 * sin_theta + g_ / resistance_full_) / resistance_full_) * decay - (g_ / resistance_full_) * t; // 完整线性阻力下t秒后的高度变化
        return { x, y }; // 返回水平距离和高度变化
    } 
};
// 弹道重现
struct Bullet { // 记录一颗已经发射出去的子弹
    TimePoint fire_time; // 这颗子弹的发射时刻
    ISO3 T_shoot2world; 
    double speed_in_world; // 子弹发射时的速度大小
    inline std::optional<Vec3> // 返回值可能为空,不为空时表示子弹在世界坐标系下的位置
    get_pos_at(TimePoint t, BallisticTrajectory::Ptr ballistic_trajectory, const std::pair<double, double>& offset)
        const { // 查询这颗子弹在某个时刻t的位置
        double dt = std::chrono::duration<double>(t - fire_time).count(); // 计算从发射到查询时刻经过了多少秒
        if (dt <= 0) { // 如果查询时间早于或等于发射时间
            return std::nullopt; // 子弹还没飞出去，返回空
        } 

        auto ypr = tools::matrix2euler(T_shoot2world.linear(), tools::EulerOrder::ZYX); // 从发射位姿的旋转矩阵里取出yaw、pitch、roll
        double yaw =ypr[0] - offset.first; // 发射yaw-yaw零点偏移，得到弹道计算使用的水平角
        double pitch = -ypr[1] - offset.second; // 弹道计算使用的俯仰角，这样的话抬头时pitch为正，v_z就是正
        auto [dis, height] =
            ballistic_trajectory->solve_distance_height(pitch, speed_in_world, dt); // 用弹道模型算dt秒后水平飞行距离和高度变化

        double x = dis * std::cos(yaw); // 把水平距离按yaw投影到world的x方向
        double y = dis * std::sin(yaw); // 把水平距离按yaw投影到world的y方向
        double z = height; // z方向直接使用弹道算出的高度变化

        return T_shoot2world.translation() + Vec3(x, y, z); // 发射点世界坐标加上飞行偏移，得到子弹当前世界坐标
    } // 单颗子弹位置查询结束
}; 

class BulletReappear { // 管理已经发射的子弹列表，并批量查询这些子弹的位置
public: 
    mutable std::mutex mtx; // 互斥锁，用来保护bullets，mutable表示这个锁可以被const成员函数使用
    std::deque<Bullet> bullets; // 保存仍在飞行、还需要追踪的子弹，队首通常是最早发射的

    BulletReappear(const YAML::Node& config) { // 创建子弹管理器时传入配置
        ballistic_trajectory = BallisticTrajectory::create(config["ballistic_trajectory"]); 
    } 

    inline void push_back(const Bullet& bullet) { // 把一颗新发射的子弹加入列表
        std::lock_guard<std::mutex> lock(mtx); // 当前作用域内自动加锁，离开函数时自动解锁
        bullets.push_back(std::move(bullet)); // 把子弹放到队列末尾
    } 

    inline void update(TimePoint t, double max_fly_time) { // 根据当前时间清理飞行太久的旧子弹
        if (bullets.empty()) // 如果队列本来就是空的
            return; // 直接返回，不需要加锁清理

        std::lock_guard<std::mutex> lock(mtx); // 清理队列前加锁，避免其他线程同时读写
        while (!bullets.empty() // 只要队列里还有子弹
               && std::chrono::duration<double>(t - bullets.front().fire_time).count() // 计算最早那颗子弹已经飞了多久
                      > max_fly_time) // 如果飞行时间超过允许保留的最大时间
        {
            bullets.pop_front(); // 删除队首这颗最老的子弹
        } 
    } 

    inline std::vector<Vec3> // 返回所有能成功计算出来的子弹位置
    get_bullet_positions(TimePoint t, const std::pair<double, double>& offset) const { // 查询所有已记录子弹在时刻t的世界坐标
        std::lock_guard<std::mutex> lock(mtx); // 读取bullets前加锁，防止和添加/清理同时发生
        std::vector<Vec3> positions; // 用来存放每颗有效子弹的位置
        for (const auto& bullet : bullets) { // 遍历当前记录的每一颗子弹
            auto p_opt = bullet.get_pos_at(t, ballistic_trajectory, offset); // 计算这颗子弹在时刻t的位置
            if (p_opt) { // 如果这颗子弹的位置成功算出来
                positions.push_back(*p_opt); // 把位置加入结果数组
            } // 单颗子弹处理结束
        } // 所有子弹遍历结束
        return positions;
    } 

    BallisticTrajectory::Ptr ballistic_trajectory; // 当前子弹管理器使用的弹道解算器
}; 
} // namespace dts_vision
