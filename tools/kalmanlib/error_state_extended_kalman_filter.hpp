#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cassert>
#include <ceres/jet.h>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace dts_vision::tools::KalmanLib {

template<int N_X, class PredicFunc>
class ErrorStateEKF {
public:
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using Jet = ceres::Jet<double, N_X>;
    using JetMatrixX1 = Eigen::Matrix<Jet, N_X, 1>;

    using UpdateQFunc = std::function<MatrixXX()>;
    using InjectFunc = std::function<void(const MatrixX1&, MatrixX1&)>;
    using BoxMinusFunc = std::function<void(const MatrixX1&, const MatrixX1&, MatrixX1&)>;
    using InjectJetFunc = std::function<void(const JetMatrixX1&, JetMatrixX1&)>;
    using BoxMinusJetFunc =
        std::function<void(const JetMatrixX1&, const JetMatrixX1&, JetMatrixX1&)>;

    ErrorStateEKF() = default;

    template<class Inject, class BoxMinus>
    explicit ErrorStateEKF(
        const PredicFunc& f,
        const UpdateQFunc& u_q,
        const Inject& inject,
        const BoxMinus& box_minus,
        const MatrixXX& P0
    ):
        f(f),
        update_Q(u_q),
        P_delta(P0) {
        set_inject_state(inject);
        set_box_minus_state(box_minus);
    }

    void set_state(const MatrixX1& x0) noexcept {
        x_nominal = x0;
        delta_x.setZero();
    }

    //! 增益掩码: 第 i 维为 0 时该误差状态不接受观测修正(冻结), 为 1 时正常更新。
    //! 用于“本帧对某个状态没有信息”的场景(例如单块平面观测对面外俯仰近似不可观测):
    //! 把对应增益行置零得到的是一个次优但一致的增益, 协方差仍按 Joseph 形式更新, 不会虚假收敛。
    void set_gain_mask(const MatrixX1& mask) noexcept {
        gain_mask = mask;
    }
    void clear_gain_mask() noexcept {
        gain_mask.setOnes();
    }

    //! 新息门控: 对每个观测做卡方检验(NIS = rᵀ S⁻¹ r, S = H P Hᵀ + R), 超阈值的观测本帧丢弃。
    //! chi2_scale 是对卡方 0.999 分位数的缩放系数: <= 0 关闭门控(与无门控时数值逐位一致),
    //! 1.0 用标准分位数, 小于 1 更严格。关联错误(灯条配到邻板)属于硬离群点,
    //! 没有门控时一次错关联就是一个硬约束, 足以把整车状态拉偏。
    void set_innovation_gate(double chi2_scale) noexcept {
        innovation_gate_scale = chi2_scale;
    }
    void clear_innovation_gate() noexcept {
        innovation_gate_scale = 0.0;
    }
    //! 上一次 update / update_multi 中被门控拒绝的观测数, 用于调参和诊断。
    [[nodiscard]] int last_rejected_obs_num() const noexcept {
        return rejected_obs_num;
    }

    //! 卡方分布 0.999 分位数。1~12 维查表, 更高维用 Wilson-Hilferty 近似(误差 < 2%)。
    static double chi2_quantile_999(int dof) noexcept {
        constexpr int TABLE_N = 12;
        constexpr double TABLE[TABLE_N] = { 10.828, 13.816, 16.266, 18.467, 20.515, 22.458,
                                            24.322, 26.124, 27.877, 29.588, 31.264, 32.909 };
        if (dof <= 0)
            return 0.0;
        if (dof <= TABLE_N)
            return TABLE[dof - 1];

        constexpr double Z_999 = 3.090232306; // 标准正态 0.999 分位数
        const double d = static_cast<double>(dof);
        const double t = 1.0 - 2.0 / (9.0 * d) + Z_999 * std::sqrt(2.0 / (9.0 * d));
        return d * t * t * t;
    }

    void set_update_Q(const UpdateQFunc& u_q) {
        update_Q = u_q;
    }
    void set_predict_func(const PredicFunc& f_) {
        f = f_;
    }
    template<class Inject>
    void set_inject_state(const Inject& inject) {
        inject_state = inject;
        inject_state_jet = inject;
    }
    template<class BoxMinus>
    void set_box_minus_state(const BoxMinus& box_minus) {
        box_minus_state = box_minus;
        box_minus_state_jet = box_minus;
    }

    MatrixX1 predict() noexcept {
        if (inject_state && box_minus_state) {
            const MatrixX1 x_prev = x_nominal;
            MatrixX1 x_pred;
            f(x_prev.data(), x_pred.data());
            x_nominal = x_pred;

            JetMatrixX1 x_nominal_jet;
            JetMatrixX1 x_prev_jet;
            for (int i = 0; i < N_X; ++i) {
                x_nominal_jet[i] = Jet(x_nominal[i]);
                x_prev_jet[i] = Jet(x_prev[i]);
            }

            JetMatrixX1 delta_jet;
            for (int i = 0; i < N_X; ++i) {
                delta_jet[i] = Jet(0.0);
                delta_jet[i].v[i] = 1.0;
            }

            JetMatrixX1 x_pert_jet = x_prev_jet;
            inject_state_jet(delta_jet, x_pert_jet);

            JetMatrixX1 x_pert_pred_jet;
            f(x_pert_jet.data(), x_pert_pred_jet.data());

            JetMatrixX1 delta_pred_jet;
            box_minus_state_jet(x_nominal_jet, x_pert_pred_jet, delta_pred_jet);
            for (int i = 0; i < N_X; ++i)
                F.row(i) = delta_pred_jet[i].v.transpose();

            delta_x = F * delta_x;

            Q = update_Q();
            P_delta = F * P_delta * F.transpose() + Q;
            P_delta = 0.5 * (P_delta + P_delta.transpose());

            return x_nominal;
        }

        std::array<ceres::Jet<double, N_X>, N_X> x_jet;

        for (int i = 0; i < N_X; ++i) {
            x_jet[i].a = x_nominal[i];
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;
        }

        std::array<ceres::Jet<double, N_X>, N_X> x_pred_jet;
        f(x_jet.data(), x_pred_jet.data());

        for (int i = 0; i < N_X; ++i) {
            x_nominal[i] = x_pred_jet[i].a;
            F.row(i) = x_pred_jet[i].v.transpose();
        }

        // error propagation
        delta_x = F * delta_x;

        Q = update_Q();
        P_delta = F * P_delta * F.transpose() + Q;

        // enforce symmetry
        P_delta = 0.5 * (P_delta + P_delta.transpose());

        return x_nominal;
    }

    template<int N_Z, class MeasureFunc, class UpdateRFunc, class ResidualFunc>
    MatrixX1 update(
        const Eigen::Matrix<double, N_Z, 1>& z,
        const MeasureFunc& h,
        const UpdateRFunc& update_R,
        const ResidualFunc& cal_residual
    ) noexcept {
        using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;
        using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
        using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
        using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;

        rejected_obs_num = 0;

        const MatrixX1 delta_0 = delta_x;
        MatrixX1 delta_iter = delta_x;
        const MatrixXX P_iter = P_delta;

        MatrixZX H;
        MatrixXZ K;
        MatrixZZ R;

        for (int iter = 0; iter < iteration_num; ++iter) {
            MatrixX1 x_eval = x_nominal;

            // inject error-state
            if (inject_state)
                inject_state(delta_iter, x_eval);
            else
                x_eval += delta_iter;

            // auto diff
            std::array<ceres::Jet<double, N_X>, N_X> x_jet;
            for (int i = 0; i < N_X; ++i) {
                x_jet[i].a = x_eval[i];
                x_jet[i].v.setZero();
                x_jet[i].v[i] = 1.0;
            }

            std::array<ceres::Jet<double, N_X>, N_Z> z_jet;
            h(x_jet.data(), z_jet.data());

            MatrixZ1 z_pred;
            for (int i = 0; i < N_Z; ++i) {
                z_pred[i] = z_jet[i].a;
                H.row(i) = z_jet[i].v.transpose();
            }

            const MatrixZ1 residual = cal_residual(z_pred, z);
            R = update_R(z);

            const MatrixZZ S = H * P_iter * H.transpose() + R;
            const auto ldlt = S.ldlt();

            // 门控只在第 0 次迭代用先验评估: 迭代后状态已经被这个观测拉过去了,
            // 再检验等于让离群点自证清白。
            if (iter == 0 && innovation_gate_scale > 0.0) {
                const double nis = residual.dot(ldlt.solve(residual));
                const double gate = innovation_gate_scale * chi2_quantile_999(N_Z);
                // 不写成 nis > gate: 这样 NaN 和负值(S 非正定)也一并拒掉。
                if (!(nis >= 0.0 && nis <= gate)) {
                    rejected_obs_num = 1;
                    return x_nominal;
                }
            }

            const MatrixXZ PHt = P_iter * H.transpose();
            K = ldlt.solve(PHt.transpose()).transpose();
            apply_gain_mask(K);

            // IEKF 的 Gauss-Newton 步: δ_{i+1} = δ_0 + K_i (r_i + H_i (δ_i - δ_0))。
            // 原来是 δ_{i+1} = δ_i + K_i r_i, 丢掉了先验拉回项, 迭代多次后收敛到纯测量
            // 最小二乘解, R 的绝对大小不再起保护作用(只剩相对权重有效), 而末尾协方差
            // 只按最后一次 K/H 收缩一次 —— 状态走了 iteration_num 次修正的距离,
            // 协方差却只按一次更新, 状态与协方差不自洽。iteration_num == 1 时两式等价。
            delta_iter = delta_0 + K * (residual + H * (delta_iter - delta_0));
        }

        if (inject_state)
            inject_state(delta_iter, x_nominal);
        else
            x_nominal += delta_iter;

        delta_x.setZero();

        const MatrixXX I = MatrixXX::Identity();
        P_delta = (I - K * H) * P_iter * (I - K * H).transpose() + K * R * K.transpose();

        P_delta = 0.5 * (P_delta + P_delta.transpose());

        return x_nominal;
    }

    struct ObsBase {
        virtual ~ObsBase() = default;
        virtual int dim() const = 0;

        virtual void predict(const Eigen::VectorXd& x, Eigen::VectorXd& z_pred) const = 0;

        virtual void
        residual_and_R(const Eigen::VectorXd& z_pred, Eigen::VectorXd& residual, Eigen::MatrixXd& R)
            const = 0;

        virtual void evaluate(
            const Eigen::VectorXd& x,
            Eigen::MatrixXd& H,
            Eigen::VectorXd& residual,
            Eigen::MatrixXd& R
        ) const = 0;
    };

    template<int N_Z, class MeasureFunc, class UpdateRFunc, class ResidualFunc>
    struct ObsImpl: public ObsBase {
        using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;

        MatrixZ1 z;
        MeasureFunc h;
        UpdateRFunc update_R;
        ResidualFunc residual_func;

        ObsImpl(
            const MatrixZ1& z_,
            const MeasureFunc& h_,
            const UpdateRFunc& r_,
            const ResidualFunc& res_
        ):
            z(z_),
            h(h_),
            update_R(r_),
            residual_func(res_) {}

        int dim() const override {
            return N_Z;
        }

        void predict(const Eigen::VectorXd& x, Eigen::VectorXd& z_pred) const override {
            assert(x.size() == N_X);

            std::array<double, N_X> x_data;
            for (int i = 0; i < N_X; ++i)
                x_data[i] = x[i];

            std::array<double, N_Z> z_data;
            h(x_data.data(), z_data.data());

            z_pred.resize(N_Z);
            for (int i = 0; i < N_Z; ++i)
                z_pred[i] = z_data[i];
        }

        void
        residual_and_R(const Eigen::VectorXd& z_pred, Eigen::VectorXd& residual, Eigen::MatrixXd& R)
            const override {
            assert(z_pred.size() == N_Z);

            MatrixZ1 z_pred_fixed;
            for (int i = 0; i < N_Z; ++i)
                z_pred_fixed[i] = z_pred[i];

            residual = residual_func(z_pred_fixed, z);
            R = update_R(z);
        }

        void evaluate(
            const Eigen::VectorXd& x,
            Eigen::MatrixXd& H,
            Eigen::VectorXd& residual,
            Eigen::MatrixXd& R
        ) const override {
            assert(x.size() == N_X);

            std::array<ceres::Jet<double, N_X>, N_X> x_jet;
            for (int i = 0; i < N_X; ++i) {
                x_jet[i].a = x[i];
                x_jet[i].v.setZero();
                x_jet[i].v[i] = 1.0;
            }

            std::array<ceres::Jet<double, N_X>, N_Z> z_jet;
            h(x_jet.data(), z_jet.data());

            H.resize(N_Z, N_X);
            residual.resize(N_Z);
            R.resize(N_Z, N_Z);

            MatrixZ1 z_pred;
            for (int i = 0; i < N_Z; ++i) {
                z_pred[i] = z_jet[i].a;
                H.row(i) = z_jet[i].v.transpose();
            }

            residual = residual_func(z_pred, z);
            R = update_R(z);
        }
    };

    template<int N_Z, class MeasureFunc, class UpdateRFunc, class ResidualFunc>
    auto make_obs(
        const Eigen::Matrix<double, N_Z, 1>& z,
        MeasureFunc&& h,
        UpdateRFunc&& r,
        ResidualFunc&& res
    ) {
        using ObsT = ObsImpl<
            N_Z,
            std::decay_t<MeasureFunc>,
            std::decay_t<UpdateRFunc>,
            std::decay_t<ResidualFunc>>;

        return std::make_shared<ObsT>(
            z,
            std::forward<MeasureFunc>(h),
            std::forward<UpdateRFunc>(r),
            std::forward<ResidualFunc>(res)
        );
    }

    MatrixX1 update_multi(const std::vector<std::shared_ptr<ObsBase>>& obs_list) noexcept {
        rejected_obs_num = 0;

        const int obs_num = static_cast<int>(obs_list.size());
        if (obs_num == 0)
            return x_nominal;

        const MatrixX1 delta_0 = delta_x;
        MatrixX1 delta_iter = delta_x;
        const MatrixXX P_iter = P_delta;

        // 有效集: 迭代开始前用先验逐个做卡方检验, 之后冻结, 迭代过程中不再增删 ——
        // 否则状态被离群点拉过去以后, 它自己反而能通过检验。
        const bool gating = innovation_gate_scale > 0.0;
        const auto n = static_cast<size_t>(obs_num);
        std::vector<bool> active(n, true);
        std::vector<Eigen::MatrixXd> cached_H(n);
        std::vector<Eigen::VectorXd> cached_r(n);
        std::vector<Eigen::MatrixXd> cached_R(n);

        if (gating) {
            MatrixX1 x_eval = x_nominal;
            if (inject_state)
                inject_state(delta_0, x_eval);
            else
                x_eval += delta_0;

            for (int k = 0; k < obs_num; ++k) {
                linearize_obs(
                    *obs_list[k], delta_0, x_eval, cached_H[k], cached_r[k], cached_R[k]
                );

                const Eigen::MatrixXd& Hk = cached_H[k];
                const Eigen::VectorXd& rk = cached_r[k];
                const Eigen::MatrixXd Sk = Hk * P_iter * Hk.transpose() + cached_R[k];

                const double nis = rk.dot(Sk.ldlt().solve(rk));
                const double gate =
                    innovation_gate_scale * chi2_quantile_999(obs_list[k]->dim());

                // 不写成 nis > gate: 这样 NaN 和负值(S 非正定)也一并拒掉。
                if (!(nis >= 0.0 && nis <= gate)) {
                    active[k] = false;
                    ++rejected_obs_num;
                }
            }
        }

        int active_dim = 0;
        for (int k = 0; k < obs_num; ++k) {
            if (active[k])
                active_dim += obs_list[k]->dim();
        }

        // 全部观测都被判为离群: 本帧不做更新, 状态和协方差保持预测值。
        // 这里必须直接返回 —— 否则会用未初始化的 K/H 去写 P_delta。
        if (active_dim == 0)
            return x_nominal;

        Eigen::MatrixXd H(active_dim, N_X);
        Eigen::VectorXd residual(active_dim);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(active_dim, active_dim);
        Eigen::MatrixXd K(N_X, active_dim);

        for (int iter = 0; iter < iteration_num; ++iter) {
            MatrixX1 x_eval = x_nominal;

            if (inject_state)
                inject_state(delta_iter, x_eval);
            else
                x_eval += delta_iter;

            int offset = 0;
            for (int k = 0; k < obs_num; ++k) {
                if (!active[k])
                    continue;

                const int d = obs_list[k]->dim();

                if (iter == 0 && gating) {
                    // 门控那一遍就是在同一个点(delta_iter == delta_0)上线性化的, 直接复用,
                    // 不必再做一次数值雅可比。
                    H.block(offset, 0, d, N_X) = cached_H[k];
                    residual.segment(offset, d) = cached_r[k];
                    R.block(offset, offset, d, d) = cached_R[k];
                } else {
                    Eigen::MatrixXd Hk, Rk;
                    Eigen::VectorXd rk;
                    linearize_obs(*obs_list[k], delta_iter, x_eval, Hk, rk, Rk);

                    H.block(offset, 0, d, N_X) = Hk;
                    residual.segment(offset, d) = rk;
                    R.block(offset, offset, d, d) = Rk;
                }

                offset += d;
            }

            const Eigen::MatrixXd S = H * P_iter * H.transpose() + R;
            const auto ldlt = S.ldlt();

            const Eigen::MatrixXd PHt = P_iter * H.transpose();
            K = ldlt.solve(PHt.transpose()).transpose();
            apply_gain_mask(K);

            // 见 update() 中同一处的说明: 补回 IEKF 的先验拉回项。
            // δ_{i+1} = δ_0 + K_i (r_i + H_i (δ_i - δ_0))
            delta_iter = delta_0 + K * (residual + H * (delta_iter - delta_0));
        }

        if (inject_state)
            inject_state(delta_iter, x_nominal);
        else
            x_nominal += delta_iter;

        delta_x.setZero();

        const MatrixXX I = MatrixXX::Identity();
        P_delta = (I - K * H) * P_iter * (I - K * H).transpose() + K * R * K.transpose();

        P_delta = 0.5 * (P_delta + P_delta.transpose());

        return x_nominal;
    }

    void set_iteration_num(int n) {
        iteration_num = std::max(1, n);
    }

private:
    //! 对单个观测做线性化, 输出 H(d×N_X)、残差 r(d) 和观测噪声 R(d×d)。
    //! x_eval 必须是 delta_eval 注入 x_nominal 之后的状态, 两者由调用方保持一致。
    //! 数值雅可比的定义与残差约定配套: r(δ) ≈ r_i - H (δ - δ_i), 故 H = -∂r/∂δ。
    void linearize_obs(
        const ObsBase& obs,
        const MatrixX1& delta_eval,
        const MatrixX1& x_eval,
        Eigen::MatrixXd& Hk,
        Eigen::VectorXd& rk,
        Eigen::MatrixXd& Rk
    ) const {
        if (!(inject_state && box_minus_state)) {
            obs.evaluate(x_eval, Hk, rk, Rk);
            return;
        }

        Eigen::VectorXd z_pred;
        obs.predict(x_eval, z_pred);
        obs.residual_and_R(z_pred, rk, Rk);

        const int d = obs.dim();
        Hk.resize(d, N_X);
        constexpr double eps = 1e-6;
        for (int i = 0; i < N_X; ++i) {
            MatrixX1 delta_plus = delta_eval;
            MatrixX1 delta_minus = delta_eval;
            delta_plus[i] += eps;
            delta_minus[i] -= eps;

            MatrixX1 x_plus = x_nominal;
            MatrixX1 x_minus = x_nominal;
            inject_state(delta_plus, x_plus);
            inject_state(delta_minus, x_minus);

            Eigen::VectorXd z_plus, z_minus;
            obs.predict(x_plus, z_plus);
            obs.predict(x_minus, z_minus);

            Eigen::VectorXd dz, unused_residual;
            Eigen::MatrixXd unused_R;
            obs.residual_and_R(z_minus, dz, unused_R);
            obs.residual_and_R(z_plus, unused_residual, unused_R);
            dz = unused_residual - dz;
            Hk.col(i) = -dz / (2.0 * eps);
        }
    }

    template<class GainMatrix>
    void apply_gain_mask(GainMatrix& k) const noexcept {
        for (int i = 0; i < N_X; ++i) {
            if (gain_mask[i] != 1.0) {
                k.row(i) *= gain_mask[i];
            }
        }
    }

    PredicFunc f;
    UpdateQFunc update_Q;
    InjectFunc inject_state;
    BoxMinusFunc box_minus_state;
    InjectJetFunc inject_state_jet;
    BoxMinusJetFunc box_minus_state_jet;

    MatrixXX F = MatrixXX::Zero();
    MatrixXX Q = MatrixXX::Zero();

    MatrixX1 x_nominal = MatrixX1::Zero();
    MatrixX1 delta_x = MatrixX1::Zero();
    MatrixX1 gain_mask = MatrixX1::Ones();
    MatrixXX P_delta = MatrixXX::Identity();

    int iteration_num = 1;

    //! <= 0 表示关闭新息门控, 此时数值行为与加门控之前完全一致。
    double innovation_gate_scale = 0.0;
    int rejected_obs_num = 0;
};

} // namespace dts_vision::tools::KalmanLib

namespace dts_vision {
namespace kalman_hybird_lib = tools::KalmanLib;
}
