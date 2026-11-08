#pragma once
#include "angles.h"
#include "tools/common/type_common.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <ceres/jet.h>
#include <cmath>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <pwd.h>
#include <regex>
#include <utility>
#include <vector>
/*SO(2)   2D旋转矩阵       平面朝向
  SE(2)   平面位姿	       差速/阿克曼底盘
  SO(3)   3D旋转矩阵       刚体姿态
  SE(3)   3D刚体位姿       机械臂末端、相机、IMU
  Sim(3)  相似变换         单目SLAM尺度漂移
  SE_2(3) 姿态、速度、位置	InEKF/IMU 导航*/
namespace dts_vision::tools {
template<class T>
inline Eigen::Matrix<T, 3, 3> so3_hat(const Eigen::Matrix<T, 3, 1>& w) { // 把 so(3) 向量转换成反对称矩阵
    Eigen::Matrix<T, 3, 3> W; // 声明反对称矩阵 W
    W << T(0), -w.z(), w.y(), w.z(), T(0), -w.x(), -w.y(), w.x(), T(0); 
    /*在三维空间中，两个向量a和b的叉积a×b可以写成矩阵乘法的形式：
        a×b=[a]_x⋅b
        其中[a]_x就是向量a的反对称矩阵,即：
        如果输入向量是w=(wx,wy,wz)^T，那么w对应的反对称矩阵W就是：
        W=[0, −wz, wy]
          [wz, 0, −wx]
          [−wy, wx, 0]
        用这个矩阵去乘任意向量v=(vx,vy,vz)^T：
        W⋅v=[0, −wz, wy] [vx] [-wzvy+wyvz]
            [wz, 0, −wx]⋅[vy]=[wzvx−wxvz]
            [−wy, wx, 0] [vz] [-wyvx+wxvy]
        恰好就是叉积w×v的结果*/
    return W; 
}
/*旋转向量（轴角）：描述的是绕空间中的某一根固定轴转了多少度
    向量的方向 = 旋转轴
    向量的长度（模长）= 旋转角度*/
template<class T> 
inline Eigen::Matrix<T, 3, 3> so3_exp(const Eigen::Matrix<T, 3, 1>& phi) { // 把 so(3) 旋转向量通过指数映射转成 SO(3) 旋转矩阵
    const T theta2 = phi.squaredNorm(); // 计算旋转向量模长的平方
    const T theta = ceres::sqrt(theta2); // 计算旋转向量模长|v|，兼容ceres自动微分类型

    const auto W = so3_hat(phi); // 计算旋转向量对应的反对称矩阵--->K
    const auto W2 = W * W; // 计算反对称矩阵的平方项--->K^2

    Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity(); // 从单位旋转矩阵开始构造结果
    /* Rodrigues 公式: R=I+K*sinθ+(1-cosθ)K^2
    其中K=[0, −kz, ky],其中k是单位旋转轴,大小指轴长，eg:(0,0,1)（指向上方，长度为1）
          [kz, 0, −kx]
          [0, −kz, ky]
    phi = [wx, wy, wz]^T
    theta = |phi| = sqrt(wx^2 + wy^2 + wz^2)
    旋转向量指的是绕轴 k = [kx, ky, kz] 一次性旋转 theta 弧度
    所以：phi = [wx, wy, wz] = theta * [kx, ky, kz]*/
    T A; // 声明 Rodrigues 公式的一阶系数 A--->sin(θ)/θ
    T B; // 声明 Rodrigues 或 SE(3) 公式中的二阶系数 B--->1-cos(θ)/θ^2
    if (theta2 < T(1e-12)) { // 如果旋转角很小，使用泰勒展开避免数值不稳定
        const T theta4 = theta2 * theta2;
        A = T(1) - theta2 / T(6) + theta4 / T(120); // sin(θ) = θ - θ^3/6 + θ^5/120
        B = T(0.5) - theta2 / T(24) + theta4 / T(720); // cos(θ) = 1 - θ^2/2 + θ^4/24 - θ^6/720
    } else { // 角度不小时使用普通三角函数公式
        A = ceres::sin(theta) / theta; // 直接计算 Rodrigues 公式 A 系数
        B = (T(1) - ceres::cos(theta)) / theta2; // 直接计算 Rodrigues 公式 B 系数
    }

    R += A * W + B * W2; // 按 Rodrigues 公式叠加一阶和二阶项

    return R; 
}

template<class T> 
inline Eigen::Transform<T, 3, Eigen::Isometry> // 声明返回 Eigen 三维等距变换的函数
se3_exp(const Eigen::Matrix<T, 3, 1>& rho, const Eigen::Matrix<T, 3, 1>& phi) { // 把 se(3) 平移/旋转增量通过指数映射转成 SE(3) 位姿
    const T theta2 = phi.squaredNorm(); // 计算旋转向量模长的平方
    const T theta = ceres::sqrt(theta2); // 计算旋转向量模长，兼容 ceres 自动微分类型

    const auto W = so3_hat(phi); // 计算旋转向量对应的反对称矩阵
    const auto W2 = W * W; // 计算反对称矩阵的平方项
    /*SE(3)的矩阵结构:[R t]
                    [0 1]
      se(3)的矩阵结构:[ϕ ρ]--->由平移/旋转增量构成
                    [0 1]             
    V矩阵：左雅可比矩阵
    为什么平移要乘V？因为一边旋转一边平移时，平移量ρ是在局部坐标系（机体坐标系）下测量的，而世界坐标系下的最终平移量需要把局部位移绕轴转动过程中每一步的微小位移都累加起来
    数学上，这个V定义为旋转矩阵exp(ϕ)对时间的积分（即左雅可比矩阵）：
    V = ∫exp(t⋅ϕ) dt--->从0到1积分
    就是想象物体在1s内完成旋转，在每一瞬间，局部位移都被当前已旋转到的姿态带到了新的方向，V做的就是把这1s内所有瞬间的方向变化积分起来
    V = ∫[I + W*sin(t0)/0 + W^2*(1 - cos(t0)/0^2] dt--->从0到1积分：
    第一项：∫1dt=1
    第二项（W的系数B）：
    B=​∫sin(t0)/0dt=1−cosθ​/θ^2
    第三项(W^2的系数)：
    C=∫(1-cos(t0)/0^2dt=(0-sinθ)/θ^3*/
    T B; 
    T C; 
    if (theta2 < T(1e-12)) { // 如果旋转角很小，使用泰勒展开避免数值不稳定
        const T theta4 = theta2 * theta2; 
        B = T(0.5) - theta2 / T(24) + theta4 / T(720); // 1-cos(θ)=θ^2/2-θ^4/24+θ^6/720
        C = T(1.0 / 6.0) - theta2 / T(120) + theta4 / T(5040); // 0-sin(θ)=θ^3/6-θ^5/120+θ^7/5040
    } else { // 角度不小时使用普通三角函数公式
        B = (T(1) - ceres::cos(theta)) / theta2; // 直接计算 Rodrigues 公式 B 系数
        C = (theta - ceres::sin(theta)) / (theta2 * theta); // 直接计算 SE(3) 左雅可比的 C 系数
    }

    const Eigen::Matrix<T, 3, 3> V = Eigen::Matrix<T, 3, 3>::Identity() + B * W + C * W2; // 计算 SE(3) 指数映射中的平移左雅可比 V

    Eigen::Transform<T, 3, Eigen::Isometry> T_se3 = Eigen::Transform<T, 3, Eigen::Isometry>::Identity(); // 把 SE(3) 结果初始化为单位变换
    T_se3.linear() = so3_exp(phi); // 用 so3_exp 计算并设置旋转部分
    T_se3.translation() = V * rho; // 用左雅可比 V 修正并设置平移部分
    return T_se3; 
}
template<class T> 
inline Eigen::Matrix<T, 3, 1> so3_log(const Eigen::Matrix<T, 3, 3>& R) { // 把SO(3)旋转矩阵通过对数映射转成so(3)旋转向量
    const T cos_theta = (R.trace() - T(1.0)) * T(0.5); // 由旋转矩阵迹计算旋转角余弦
    /*R=I+K*sinθ+(1-cosθ)K^2
    利用迹（Trace）提取旋转角0:
        对矩阵R取迹（对角线元素之和），利用反对称矩阵迹为零tr(K) = 0和K^2的迹为-2，可以得到：
        tr(R) = 3 + 0*sinθ - 2(1 - cosθ) = 1 + 2cosθ
        因此：cos0 = [tr(R) - 1]/2
    利用反对称部分提取旋转轴u与sinθ
        计算R - R^T（矩阵转置相减）：
        R - R^T = [ I + K*sinθ + (1-cosθ)*K^2] - [ I - K*sinθ + (1-cosθ)*K^2] = 2K*sinθ--->2*W*sinθ/θ
                    [0, R(0,1)-R(1,0), R(0,2)-R(2,0)]            [0, −wz, wy]
        即R - R^T = [R(1,0)-R(0,1), 0, R(1,2)-R(2,1)]=2*W*sinθ/θ=[wz, 0, −wx]*2*sinθ/θ
                    [R(2,0)-R(0,2), R(2,1)-R(1,2), 0]            [−wy, wx, 0]*/
    Eigen::Matrix<T, 3, 1> w; 
    w << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1); // 注意这个w=2*W*sinθ/θ
    if (cos_theta > T(1.0 - 1e-12)) { // 当旋转角接近0时走小角度近似
        return T(0.5) * w; // 此时θ/(2*sinθ)用等价无穷小就是0.5
    }
    const T theta = ceres::acos(cos_theta); // 根据 cos(theta) 求旋转角
    const T scale = theta / (T(2.0) * ceres::sin(theta)); // 真正的w_true=w*θ/(2*sinθ)
    return scale * w; // 返回旋转向量
}
template<class T> 
inline void se3_log( // 声明 SE(3) 对数映射函数
    const Eigen::Transform<T, 3, Eigen::Isometry>& T_se3, // 传入待求对数的 SE(3) 位姿
    Eigen::Matrix<T, 3, 1>& rho, // 输出平移向量
    Eigen::Matrix<T, 3, 1>& phi // 输出旋转向量
) { 
    phi = so3_log(T_se3.linear().eval()); // 先对旋转矩阵求 so(3) 对数--->把 SO(3) 旋转矩阵通过对数映射转成 so(3) 旋转向量
    /*旋转部分的提取--->直接调用so3_log得到旋转向量
      平移部分：
        平移向量t与局部平移增量ρ的关系是：t=V*ρ
        其中左雅可比矩阵V的表达式为:V = I + B⋅W + C⋅W^2
        B=1−cosθ​/θ^2
        C=(0-sinθ)/θ^3
        那么为了求出ρ，就要计算V的逆矩阵：ρ = V^{-1}⋅t
        因为W满足循环性质W^3 = -θ^2⋅W，所以V^{-1}一定可以写成相同基底{I, W, W^2}的线性组合：
        V^{-1} = I + a⋅W + b⋅W^2
        用V去乘V^{-1}，结果必等于单位矩阵I：
        (I + B W + C W^2)(I + a W + b W^2) = I
        展开并利用W^3 = -θ^2 W和W^4 = -θa^2 W^2整理，得到关于W和W^2的二元方程组：
        对于W的系数:a + B - θ^2(Bb + Ca) = 0
        对于W^2的系数:b + Ba + C - θ^2Cb = 0 
        解得到唯一的解析解：
        a = -0.5
        b = 1/θ^2-(1+cosθ)/(2*θ*sinθ)--->D*/
    const T theta2 = phi.squaredNorm(); // 计算旋转向量模长的平方
    const T theta = ceres::sqrt(theta2); // 计算旋转向量模长，兼容 ceres 自动微分类型
    const auto W = so3_hat(phi); // 计算旋转向量对应的反对称矩阵
    const auto W2 = W * W; // 计算反对称矩阵的平方项

    T D; 
    if (theta2 < T(1e-12)) { // 如果旋转角很小，使用泰勒展开避免数值不稳定
        const T theta4 = theta2 * theta2; 
        D = T(1.0 / 12.0) + theta2 / T(720) + theta4 / T(30240); // 小角度下用泰勒展开计算 V 逆的 D 系数
    } else { // 角度不小时使用普通三角函数公式
        D = T(1) / theta2 - (T(1) + ceres::cos(theta)) / (T(2) * theta * ceres::sin(theta));
    }

    const Eigen::Matrix<T, 3, 3> V_inv = Eigen::Matrix<T, 3, 3>::Identity() - T(0.5) * W + D * W2; // 计算 SE(3) 左雅可比逆
    rho = V_inv * T_se3.translation(); // 用 V 逆把平移部分还原为李代数 rho
}
enum class EulerOrder { XYZ, XZY, YXZ, YZX, ZXY, ZYX }; // 枚举不同欧拉角旋转顺序 
template<class T>
inline Eigen::Matrix<T, 3, 1> axis_vec(int axis) { // 根据0/1/2返回X/Y/Z轴单位向量 
    Eigen::Matrix<T, 3, 1> axis_vector = Eigen::Matrix<T, 3, 1>::Zero();
    switch (axis) {  // 根据轴编号选择对应单位向量
        case 0:  
            axis_vector.x() = T(1);
            return axis_vector;
        case 1:  
            axis_vector.y() = T(1);
            return axis_vector;
        case 2:  
            axis_vector.z() = T(1);
            return axis_vector;
        default:  
            throw std::invalid_argument("Invalid axis");
    } 
} 

inline std::array<int, 3> get_axes(EulerOrder order) { // 把欧拉角顺序枚举转换成三个轴编号
    switch (order) { // 根据不同欧拉角顺序返回不同轴数组 
        case EulerOrder::XYZ:  
            return { 0, 1, 2 };  
        case EulerOrder::XZY:  
            return { 0, 2, 1 }; 
        case EulerOrder::YXZ: 
            return { 1, 0, 2 }; 
        case EulerOrder::YZX:  
            return { 1, 2, 0 };
        case EulerOrder::ZXY:  
            return { 2, 0, 1 };  
        case EulerOrder::ZYX:  
            return { 2, 1, 0 }; 
        default:  
            throw std::invalid_argument("Unsupported EulerOrder");
    } 
}
template<class T>
inline Eigen::Quaternion<T> euler2quat(
    const Eigen::Matrix<T, 3, 1>& angles,
    EulerOrder order
) { // 输入三个欧拉角和旋转顺序，输出四元数
    auto axes = get_axes(order); // 获取旋转轴顺序 
    /*右乘：q = q * dq 
      新增旋转dq发生在当前自身坐标系/局部坐标系里 
    左乘：q = dq * q 
      新增旋转dq发生在世界坐标系/父坐标系euler2q里*/ 
    Eigen::Quaternion<T> q = Eigen::Quaternion<T>::Identity();// 初始化单位四元数作为旋转累积起点
    for (int i = 0; i < 3; ++i) { // 依次处理三个旋转角 
        q = q * Eigen::Quaternion<T>(
            Eigen::AngleAxis<T>(angles[i], axis_vec<T>(axes[i]))
        ); // 右乘依次应用旋转
    } 
    q.normalize(); // 归一化四元数
    return q; 
}

template<class T> // 模板参数T是矩阵元素类型
inline Eigen::Matrix<T, 3, 3> euler2matrix(
    const Eigen::Matrix<T, 3, 1>& angles,
    EulerOrder order
) { // 将欧拉角转换为3x3旋转矩阵
    return euler2quat(angles, order).toRotationMatrix(); // 先转四元数再转矩阵 
} 

inline Vec3 matrix2euler(const Mat3& R, EulerOrder order) { // 将旋转矩阵按指定顺序分解成欧拉角
    Vec3 angles;  

    switch (order) {  
        /*旋转矩阵的构造逻辑： 
            先绕装甲板坐标系X轴旋转Roll角（翻滚），对应旋转矩阵Rx(roll): 
            [1, 0,                0] 
            [0, cos_roll, -sin_roll] 
            [0, sin_roll,  cos_roll] 
            再绕装甲板坐标系Y轴旋转Pitch角（俯仰），对应旋转矩阵Ry(pitch): 
            [cos_pitch, 0, sin_pitch] 
            [0,         1, 0        ] 
            [-sin_pitch,0, cos_pitch] 
            再绕装甲板坐标系Z轴旋转Yaw角（偏航），对应旋转矩阵Rz(yaw): 
            [cos_yaw, -sin_yaw, 0] 
            [sin_yaw,  cos_yaw, 0] 
            [0,        0,       1] 
            复合旋转矩阵为 R = Rx(roll) * Ry(pitch) * Rz(yaw) 
            R = [cos_yaw*cos_pitch,                            -cos_pitch*sin_yaw,                                     sin_pitch] 
                [sin_roll*sin_pitch*cos_yaw+cos_roll*sin_yaw,  -sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw, -sin_roll*cos_pitch] 
                [-sin_pitch*cos_roll*cos_yaw+sin_roll*sin_yaw, sin_pitch*cos_roll*sin_yaw+sin_roll*cos_yaw,   cos_roll*cos_pitch]*/ 
        case EulerOrder::XYZ: {  
            double sin_pitch = R(0, 2);  
            if (std::abs(sin_pitch) < 1.0 - 1e-6) { // 非万向锁情况
                angles[1] = std::asin(sin_pitch); // pitch
                angles[0] = std::atan2(-R(0, 1), R(0, 0));// yaw
                angles[2] = std::atan2(-R(1, 2), R(2, 2));// roll 
            } else { // 万向锁 
                angles[1] = std::asin(sin_pitch); 
                angles[0] = std::atan2(R(1, 0), R(1, 1)); // yaw 
                angles[2] = 0;// roll--->cos_roll=1,sin_roll=0,R(0, 1)=-sin_yaw,R(1, 1)=cos_yaw 
            }
            break;  
        } 
        /*旋转矩阵的构造逻辑： 
            先绕装甲板坐标系Z轴旋转Yaw角（偏航），对应旋转矩阵Rz(yaw): 
            [cos_yaw, -sin_yaw, 0] 
            [sin_yaw,  cos_yaw, 0] 
            [0,        0,       1] 
            再绕装甲板坐标系Y轴旋转Pitch角（俯仰），对应旋转矩阵Ry(pitch): 
            [cos_pitch, 0, sin_pitch] 
            [0,         1, 0        ] 
            [-sin_pitch,0, cos_pitch] 
            再绕装甲板坐标系X轴旋转Roll角（翻滚），对应旋转矩阵Rx(roll): 
            [1, 0,                0] 
            [0, cos_roll, -sin_roll] 
            [0, sin_roll,  cos_roll] 
            复合旋转矩阵为 R = Rz(yaw) * Ry(pitch) * Rx(roll) 
            R = [cos_yaw*cos_pitch, -cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll, cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll] 
                [sin_yaw*cos_pitch, cos_yaw*cos_roll + sin_yaw*sin_pitch*sin_roll, -cos_yaw*sin_roll + sin_yaw*sin_pitch*cos_roll] 
                [-sin_pitch,        sin_roll*cos_pitch,                                                        cos_pitch*cos_roll]*/ 
        case EulerOrder::ZYX: {  
            double sin_pitch = -R(2, 0); 
            if (std::abs(sin_pitch) < 1.0 - 1e-6) { 
                angles[1] = std::asin(sin_pitch); // pitch 
                angles[0] = std::atan2(R(1, 0), R(0, 0)); // yaw 
                angles[2] = std::atan2(R(2, 1), R(2, 2)); // roll 
            } else { // 万向锁情况，yaw和roll耦合，不能同时唯一恢复
                angles[1] = std::asin(sin_pitch); // pitch仍然可恢复
                angles[0] = std::atan2(-R(0, 1), R(1, 1)); // yaw 
                angles[2] = 0; // roll--->cos_roll=1,sin_roll=0,R(0, 1)=-sin_yaw,R(1, 1)=cos_yaw
            }
            break; // 结束当前 switch 分支
        } 
        /*在万向锁时ZYX欧拉角分解不再唯一，也就是说：旋转矩阵还能确定，但yaw和roll这两个角已经耦合在一起了，没法单独唯一恢复 
        angles[2] = 0;                  
        angles[0] = atan2(...);        
        这不是“唯一正确答案”，而是：在奇异情况下，选一个合理、连续、可用的解 
        当：pitch = ±pi/2,也就是：sin_pitch = ±1时，就进入奇异状态 
        判断：if (std::abs(sin_pitch) < 1.0 - 1e-6)，就是判断是否进入奇异状态 
        为什么pitch = ±90°会出问题 
        因为这时两个旋转轴对齐了，导致自由度丢失--->第三步要绕的当前x轴已经和第一步的z轴对齐了 
        于是： 
        第一转yaw：绕z 
        第三转roll：绕当前x 
        这两个旋转实际上绕的是同一条空间轴 
        所以它们可以互相替代一部分效果 
        - 原本有3个角：yaw,pitch,roll 
        - 但当pitch到±90°时 
        - yaw和roll的效果部分重合 
        于是： 
        不同的yaw和roll组合，可能产生同一个旋转矩阵，所以无法从R唯一反推出欧拉角各自是多少*/ 

        case EulerOrder::XZY: { 
            double sz = R(1, 0); 
            if (std::abs(sz) < 1.0 - 1e-6) { 
                angles[2] = std::asin(sz);  
                angles[0] = std::atan2(-R(1, 2), R(1, 1));  
                angles[1] = std::atan2(-R(2, 0), R(0, 0));  
            } else {  
                angles[2] = std::asin(sz);  
                angles[0] = std::atan2(R(2, 1), R(2, 2));  
                angles[1] = 0;  
            }
            break;
        } 
        case EulerOrder::YXZ: {  
            double sx = -R(1, 2);  
            if (std::abs(sx) < 1.0 - 1e-6) {  
                angles[0] = std::asin(sx); 
                angles[1] = std::atan2(R(0, 2), R(2, 2));  
                angles[2] = std::atan2(R(1, 0), R(1, 1));  
            } else { 
                angles[0] = std::asin(sx);  
                angles[1] = std::atan2(-R(2, 0), R(0, 0));  
                angles[2] = 0; 
            }
            break;  
        }

        case EulerOrder::YZX: {  
            double sz = -R(0, 1);  
            if (std::abs(sz) < 1.0 - 1e-6) {  
                angles[2] = std::asin(sz);  
                angles[1] = std::atan2(R(0, 2), R(0, 0)); 
                angles[0] = std::atan2(R(2, 1), R(1, 1)); 
            } else { 
                angles[2] = std::asin(sz); 
                angles[1] = std::atan2(-R(2, 0), R(2, 2)); 
                angles[0] = 0;  
            }
            break; 
        } 

        case EulerOrder::ZXY: {  
            double sx = R(2, 1);  
            if (std::abs(sx) < 1.0 - 1e-6) { 
                angles[0] = std::asin(sx);  
                angles[2] = std::atan2(-R(0, 1), R(1, 1));  
                angles[1] = std::atan2(-R(2, 0), R(2, 2));  
            } else {  
                angles[0] = std::asin(sx); 
                angles[2] = std::atan2(R(1, 0), R(0, 0)); 
                angles[1] = 0;  
            }
            break; 
        }

        default: 
            throw std::invalid_argument("Unsupported EulerOrder"); 
    }

    return angles; 
} 

inline Vec3 quat2euler(const Quaternion& q, EulerOrder order) { // 将四元数按指定顺序转成欧拉角 
    return matrix2euler(q.toRotationMatrix(), order); // 先把四元数转旋转矩阵，再复用matrix2euler 
} 

inline std::string expand_env(const std::string& s) {
    std::regex env_re(R"(\$\{([^}]+)\})");
    std::smatch match;
    std::string result = s;
    while (std::regex_search(result, match, env_re)) {
        const char* env = std::getenv(match[1].str().c_str());
        std::string val = env ? env : "";
        result.replace(match.position(0), match.length(0), val);
    }
    return result;
}
template<typename Func>
void dt_once(Func&& func, std::chrono::duration<double> dt) noexcept {
    static auto last_call = std::chrono::steady_clock::now();// 调用初记一个，然后不变了

    auto now = std::chrono::steady_clock::now();
    if (now - last_call >= dt) {
        last_call = now;
        func();
    }
}
template<typename T>
concept Point2DLike = requires(T p) {
    {
        p.x
        } -> std::convertible_to<float>;
    {
        p.y
        } -> std::convertible_to<float>;
    T { 0.f, 0.f };
};
template<Point2DLike T>
[[nodiscard]] inline T transform_point2D(const Eigen::Matrix3f& H, const T& p) noexcept {
    const Eigen::Vector3f hp { p.x, p.y, 1.f };
    const Eigen::Vector3f tp = H * hp;
    return { tp.x(), tp.y() };
}
inline cv::Rect2f transform_rect(const Eigen::Matrix3f& H, const cv::Rect2f& rect) {
    cv::Point2f p1(rect.x, rect.y);
    cv::Point2f p2(rect.x + rect.width, rect.y);
    cv::Point2f p3(rect.x, rect.y + rect.height);
    cv::Point2f p4(rect.x + rect.width, rect.y + rect.height);

    auto tp1 = tools::transform_point2D(H, p1);
    auto tp2 = tools::transform_point2D(H, p2);
    auto tp3 = tools::transform_point2D(H, p3);
    auto tp4 = tools::transform_point2D(H, p4);

    float min_x = std::min({ tp1.x, tp2.x, tp3.x, tp4.x });
    float min_y = std::min({ tp1.y, tp2.y, tp3.y, tp4.y });
    float max_x = std::max({ tp1.x, tp2.x, tp3.x, tp4.x });
    float max_y = std::max({ tp1.y, tp2.y, tp3.y, tp4.y });

    return cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
}
inline cv::Mat letterbox(
    const cv::Mat& img,
    Eigen::Matrix3f& transform_matrix,
    const int new_shape_w,
    const int new_shape_h
) noexcept {
    const int img_h = img.rows;
    const int img_w = img.cols;

    const float scale = std::min((float)new_shape_h / img_h, (float)new_shape_w / img_w);
    const int resize_h = int(img_h * scale + 0.5f);
    const int resize_w = int(img_w * scale + 0.5f);

    const int pad_h = new_shape_h - resize_h;
    const int pad_w = new_shape_w - resize_w;
    const int top = pad_h / 2;
    const int left = pad_w / 2;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(resize_w, resize_h), 0, 0, cv::INTER_LINEAR);

    cv::Mat out;
    if (pad_h == 0 && pad_w == 0) {
        out = resized;
    } else {
        cv::copyMakeBorder(
            resized,
            out,
            top,
            pad_h - top,
            left,
            pad_w - left,
            cv::BORDER_CONSTANT,
            cv::Scalar(114, 114, 114)
        );
    }

    const float inv_scale = 1.0f / scale;

    transform_matrix << inv_scale, 0, -left * inv_scale, 0, inv_scale, -top * inv_scale, 0, 0, 1;

    return out;
}
inline std::string to_upper(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::toupper(c); });
    return str;
}
template<std::size_t N1, std::size_t N2>
consteval auto concat(const char (&a)[N1], const char (&b)[N2]) {// 合并两个字符串字面量
    std::array<char, N1 + N2 - 1> result {}; // -1 是因为两个字面量都有 '\0'
    for (std::size_t i = 0; i < N1 - 1; ++i)
        result[i] = a[i];
    for (std::size_t i = 0; i < N2; ++i)
        result[i + N1 - 1] = b[i]; // 包含 '\0'
    return result;
}
template<typename T>
inline T from_vector(const std::vector<uint8_t>& data) {
    T packet {};
    std::memcpy(&packet, data.data(), sizeof(T));
    return packet;
}

template<typename T>
inline std::vector<uint8_t> to_vector(const T& data) {
    std::vector<uint8_t> packet(sizeof(T));
    std::memcpy(packet.data(), &data, sizeof(T));
    return packet;
}
inline std::pair<cv::Mat, cv::Mat> eigen_iso3_to_rvec_tvec(const ISO3& iso3) {
    cv::Mat rvec, R_cv;
    Mat3 R = iso3.linear();
    cv::eigen2cv(R, R_cv);
    cv::Rodrigues(R_cv, rvec);
    auto t = iso3.translation();
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << t.x(), t.y(), t.z());
    return { std::move(rvec), std::move(tvec) };
}
inline ISO3 rvec_tvec_to_eigen_iso3(const cv::Mat& rvec, const cv::Mat& tvec) {
    ISO3 iso3;
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    Mat3 R_eigen;
    cv::cv2eigen(R_cv, R_eigen);
    iso3.linear() = R_eigen;
    Vec3 t_eigen;
    cv::cv2eigen(tvec, t_eigen);
    iso3.translation() = t_eigen;
    return iso3;
}
inline std::vector<cv::Point2f> reprojection(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const std::vector<cv::Point3f>& object_points,// 待投影的3D点
    const ISO3& T_pose2camera_cv
) noexcept {
    auto [rvec, tvec] = eigen_iso3_to_rvec_tvec(T_pose2camera_cv);
    std::vector<cv::Point2f> pts_2d;
    pts_2d.reserve(object_points.size());
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, pts_2d);
    return pts_2d;
}
template<typename ImgPoints>
inline ImgPoints undistort_points(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const ImgPoints& img_points
) {
    ImgPoints norm_pts;
    cv::undistortPoints(img_points, norm_pts, camera_matrix, dist_coeffs);
    return norm_pts;
}
template<typename ImgPoint>
inline ImgPoint undistort_point(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const ImgPoint& img_point
) {
    std::vector<ImgPoint> norm_pts;
    cv::undistortPoints(std::vector<ImgPoint> { img_point }, norm_pts, camera_matrix, dist_coeffs);
    return norm_pts.front();
}
template<typename T>
inline void project_points_jets(
    const std::vector<cv::Point3f>& obj_pts,
    const Eigen::Transform<T, 3, Eigen::Isometry>& pose_cam,
    const cv::Mat& K,
    const cv::Mat& dist_coeffs,
    std::vector<Eigen::Matrix<T, 2, 1>>& img_pts_jet
) {
    if (obj_pts.empty())
        return;
    if (K.empty() || K.rows != 3 || K.cols != 3)
        throw std::runtime_error("Invalid K");
    if (dist_coeffs.empty())
        throw std::runtime_error("Invalid dist_coeffs");

    const Eigen::Matrix<T, 3, 3>& R = pose_cam.linear();
    const Eigen::Matrix<T, 3, 1>& t = pose_cam.translation();

    const T fx = T(K.at<double>(0, 0));
    const T fy = T(K.at<double>(1, 1));
    const T cx = T(K.at<double>(0, 2));
    const T cy = T(K.at<double>(1, 2));

    auto get_dist = [&](int i) -> double {
        return (dist_coeffs.rows == 1) ? dist_coeffs.at<double>(0, i)
                                       : dist_coeffs.at<double>(i, 0);
    };

    const int n_dist = dist_coeffs.rows * dist_coeffs.cols;
    const T k1 = n_dist > 0 ? T(get_dist(0)) : T(0);
    const T k2 = n_dist > 1 ? T(get_dist(1)) : T(0);
    const T p1 = n_dist > 2 ? T(get_dist(2)) : T(0);
    const T p2 = n_dist > 3 ? T(get_dist(3)) : T(0);
    const T k3 = n_dist > 4 ? T(get_dist(4)) : T(0);

    img_pts_jet.clear();
    img_pts_jet.reserve(obj_pts.size());

    for (const auto& pt3: obj_pts) {
        Eigen::Matrix<T, 3, 1> Pw(T(pt3.x), T(pt3.y), T(pt3.z));
        Eigen::Matrix<T, 3, 1> Pc = R * Pw + t;
        T Xc = Pc(0), Yc = Pc(1), Zc = Pc(2);
        T xp = Xc / Zc;
        T yp = Yc / Zc;

        T r2 = xp * xp + yp * yp;
        T r4 = r2 * r2;
        T r6 = r4 * r2;

        T radial = T(1) + k1 * r2 + k2 * r4 + k3 * r6;
        T xd = xp * radial + T(2) * p1 * xp * yp + p2 * (r2 + T(2) * xp * xp);
        T yd = yp * radial + p1 * (r2 + T(2) * yp * yp) + T(2) * p2 * xp * yp;

        T u = fx * xd + cx;
        T v = fy * yd + cy;

        img_pts_jet.emplace_back(u, v);
    }
}
template<typename T>
inline void project_points_jets_normalized(
    const std::vector<cv::Point3f>& obj_pts,
    const Eigen::Transform<T, 3, Eigen::Isometry>& pose_cam,
    const cv::Mat& dist_coeffs,
    std::vector<Eigen::Matrix<T, 2, 1>>& norm_pts
) {
    if (obj_pts.empty())
        return;

    if (dist_coeffs.empty())
        throw std::runtime_error("Invalid dist_coeffs");

    const Eigen::Matrix<T, 3, 3>& R = pose_cam.linear();
    const Eigen::Matrix<T, 3, 1>& t = pose_cam.translation();

    auto get_dist = [&](int i) -> double {
        return (dist_coeffs.rows == 1) ? dist_coeffs.at<double>(0, i)
                                       : dist_coeffs.at<double>(i, 0);
    };

    const int n_dist = dist_coeffs.rows * dist_coeffs.cols;

    const T k1 = n_dist > 0 ? T(get_dist(0)) : T(0);
    const T k2 = n_dist > 1 ? T(get_dist(1)) : T(0);
    const T p1 = n_dist > 2 ? T(get_dist(2)) : T(0);
    const T p2 = n_dist > 3 ? T(get_dist(3)) : T(0);
    const T k3 = n_dist > 4 ? T(get_dist(4)) : T(0);

    norm_pts.clear();
    norm_pts.reserve(obj_pts.size());

    for (const auto& pt3: obj_pts) {
        Eigen::Matrix<T, 3, 1> Pw(T(pt3.x), T(pt3.y), T(pt3.z));
        Eigen::Matrix<T, 3, 1> Pc = R * Pw + t;

        T Xc = Pc(0);
        T Yc = Pc(1);
        T Zc = Pc(2);

        T x = Xc / Zc;
        T y = Yc / Zc;

        T r2 = x * x + y * y;
        T r4 = r2 * r2;
        T r6 = r4 * r2;

        T radial = T(1) + k1 * r2 + k2 * r4 + k3 * r6;

        T xd = x * radial + T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
        T yd = y * radial + p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;

        norm_pts.emplace_back(xd, yd);
    }
}
[[nodiscard]] inline double lerp_angle(double a0, double a1, double t) noexcept {
    double d = std::remainder(a1 - a0, 2.0 * M_PI);
    return a0 + t * d;
}
[[nodiscard]] inline Vec3 load_vec3(const YAML::Node& node) {
    auto vec = node.as<std::vector<double>>();
    return Vec3(vec[0], vec[1], vec[2]);
}
[[nodiscard]] inline Mat3 load_mat3(const YAML::Node& node) {
    Mat3 result;

    if (node.IsSequence() && node.size() == 9) {
        // 一维数组
        auto vec = node.as<std::vector<double>>();
        for (int i = 0; i < 9; ++i) {
            result(i / 3, i % 3) = vec[i];
        }
    } else {
        // 二维数组
        auto mat = node.as<std::vector<std::vector<double>>>();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result(i, j) = mat[i][j];
            }
        }
    }

    return result;
}
[[nodiscard]] inline ISO3 load_isometry3(const YAML::Node& node) {
    auto trans = load_vec3(node["t"]);
    auto rot = load_mat3(node["R"]);
    ISO3 result = ISO3::Identity();
    result.translation() = trans;
    result.linear() = rot;
    return result;
}
[[nodiscard]] inline cv::Rect2f load_rect2f(const YAML::Node& node) {
    auto x = node["x"].as<double>();
    auto y = node["y"].as<double>();
    auto w = node["w"].as<double>();
    auto h = node["h"].as<double>();
    return cv::Rect2f(x, y, w, h);
}
inline std::optional<std::string> get_arg(int i, int argc, char* argv[]) {
    if (i < argc) {
        std::cout << "get args " << std::string(argv[i]) << std::endl;
        return std::make_optional(std::string(argv[i]));// 把这个参数转成string返回
    }
    return std::nullopt;
};
template<class LandMark, class ObjectPoint>
inline ISO3 solve_pnp(
    const LandMark& landmarks,
    const ObjectPoint& object_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients,
    int flags = cv::SOLVEPNP_ITERATIVE
) {
    cv::Mat rvec, tvec;
    cv::solvePnP(
        object_points,
        landmarks,
        camera_matrix,
        distortion_coefficients,
        rvec,
        tvec,
        false,
        flags
    );
    return rvec_tvec_to_eigen_iso3(rvec, tvec);
}
inline auto
calculate_distance_to_img_center(const cv::Point2f& image_point, const cv::Mat& camera_matrix) {
    auto cx = camera_matrix.at<double>(0, 2);
    auto cy = camera_matrix.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}
template<class Mat>
inline void
fill_constant_accel_noise(Mat& q, int pos_idx, int vel_idx, double noise, double dt) noexcept {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    q(pos_idx, pos_idx) = dt4 * 0.25 * noise;
    q(pos_idx, vel_idx) = dt3 * 0.5 * noise;
    q(vel_idx, pos_idx) = dt3 * 0.5 * noise;
    q(vel_idx, vel_idx) = dt2 * noise;
}
[[nodiscard]] inline double sigmoid(double x) noexcept {
    return x >= 0 ? 1.0 / (1.0 + std::exp(-x)) : std::exp(x) / (1.0 + std::exp(x));
}

[[nodiscard]] inline float rect_ioU(const cv::Rect2f& a, const cv::Rect2f& b) noexcept {
    const cv::Rect2f inter = a & b;
    const float inter_area = inter.area();
    const float union_area = a.area() + b.area() - inter_area;
    if (union_area <= 0.f || std::isnan(union_area))
        return 0.f;
    return inter_area / union_area;
}
template<typename Func, typename T>
[[nodiscard]] inline T
golden_section_search(Func&& f, T left, T right, T eps = static_cast<T>(1e-4)) {
    static_assert(std::is_floating_point_v<T>);

    constexpr T phi = static_cast<T>(0.6180339887498948482);

    T x1 = right - phi * (right - left);
    T x2 = left + phi * (right - left);

    T f1 = f(x1);
    T f2 = f(x2);

    while ((right - left) > eps) {
        if (f1 > f2) {
            left = x1;

            x1 = x2;
            f1 = f2;

            x2 = left + phi * (right - left);
            f2 = f(x2);
        } else {
            right = x2;

            x2 = x1;
            f2 = f1;

            x1 = right - phi * (right - left);
            f1 = f(x1);
        }
    }

    return (f1 < f2) ? x1 : x2;
}
template<typename RectType>
inline RectType
expand_and_clip_rect(const RectType& rect, double expand_ratio, const cv::Size& img_size) {
    RectType r = rect;
    r.x -= (r.width * (expand_ratio - 1.0) * 0.5);
    r.y -= (r.height * (expand_ratio - 1.0) * 0.5);
    r.width = (r.width * expand_ratio);
    r.height = (r.height * expand_ratio);
    RectType img_rect(0, 0, img_size.width, img_size.height);
    r = r & img_rect;
    return r;
}

template<typename T>
bool is_finite(const T& s) {
    return std::apply([](auto... v) { return (std::isfinite(v) && ...); }, s.to_tuple());
}
} // namespace dts_vision::tools

namespace dts_vision {
namespace utils = tools;
}
