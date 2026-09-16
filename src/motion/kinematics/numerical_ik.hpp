#pragma once

#include "motion/geometry/pose_error.hpp"
#include "motion/kinematics/geometric_jacobian.hpp"
#include "motion/kinematics/joint_position_constraints.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace robot::motion
{

    /** 求解停止原因。只有 Converged 表示目标位姿已通过位置和角度验收。 */
    enum class IkStatus
    {
        Converged,
        InvalidInput,
        SeedOutOfBounds,
        IterationLimit,
        TimeLimit,
        NoProgress,
        SubproblemLimit,
        NumericalFailure
    };

    /**
     * 上游按任务要求填写；默认数值只是初始参数，不是真机精度/安全承诺。
     * tolerance 是成功验收阈值；scale 是误差加权尺度，二者不是同一个概念。
     * step 是一次数值迭代的位置修正上限，不是电机速度或运动轨迹采样周期。
     */
    struct IkOptions
    {
        double position_tolerance{1e-4};   // m
        double rotation_tolerance{1e-3};   // rad
        double position_scale{0.1};        // m，位置误差除以此值后参与代价
        double rotation_scale{1.0};        // rad
        double max_rotational_step{0.2};   // rad / 数值迭代
        double max_prismatic_step{0.02};   // m / 数值迭代
        double initial_damping{1e-2};      // 无量纲，允许 [1e-6, 1e3]
        double subproblem_tolerance{1e-8}; // 归一化变量的投影梯度映射阈值
        std::size_t max_iterations{200};
        std::size_t max_damping_trials{6};
        std::size_t max_backtracks{12};
        std::size_t max_coordinate_sweeps{100};
        std::chrono::duration<double> time_budget{0.2}; // 秒，协作式检查，不是硬实时中断
    };

    /**
     * candidate_q 仅在成功评估过可行候选后有值；失败时可用于诊断，不能当成功解下发。
     * 保存最后接受的候选（代价单调下降），不是声称全局最优。
     * pose_evaluations 统计本函数直接做的 FK+误差评估，不包含雅可比内部的 FK。
     */
    template <std::size_t N>
    struct IkResult
    {
        IkStatus status{IkStatus::InvalidInput};
        std::optional<std::array<double, N>> candidate_q;
        double position_error{std::numeric_limits<double>::infinity()};
        double rotation_error{std::numeric_limits<double>::infinity()};
        std::size_t iterations{0};
        std::size_t pose_evaluations{0};
        std::chrono::duration<double> elapsed{};
        std::string message;

        /** 上游检查返回结果时调用；不以 candidate_q 是否有值代替成功判定。 */
        [[nodiscard]] bool converged() const noexcept
        {
            return status == IkStatus::Converged;
        }
    };

    namespace numerical_ik_detail
    {
        using Clock = std::chrono::steady_clock;

        /** solveNumericalIk/子问题调用：检查本次调用的累计墙钟时间，不暂停或创建线程。 */
        inline bool expired(const Clock::time_point start, const IkOptions &o)
        {
            return std::chrono::duration<double>(Clock::now() - start) >= o.time_budget;
        }

        /** 仅在求解入口检查参数；不接受 NaN、无穷、零/负尺度或空预算。 */
        inline bool validOptions(const IkOptions &o)
        {
            const double positive_values[]{
                o.position_tolerance, o.rotation_tolerance, o.position_scale, o.rotation_scale,
                o.max_rotational_step, o.max_prismatic_step, o.subproblem_tolerance,
                o.time_budget.count()};
            for (double v : positive_values)
                if (!std::isfinite(v) || v <= 0.0)
                    return false;
            return std::isfinite(o.initial_damping) && o.initial_damping >= 1e-6 &&
                   o.initial_damping <= 1e3 && o.subproblem_tolerance < 1.0 &&
                   o.max_iterations > 0 && o.max_damping_trials > 0 &&
                   o.max_backtracks > 0 && o.max_coordinate_sweeps > 0;
        }

        /**
         * solveNumericalIk 调用：旋转误差 r=Log(R_target*R_current^T) 的右雅可比逆。
         * 返回 3×3 修正矩阵 C，使固定目标下 dr ≈ -C*Jw*dq。
         * C = I + [r]x/2 + beta*[r]x^2；[r]x*v = r.cross(v)。
         * 小角度用级数避免消去误差；半角公式避免在 pi 附近除以 sin(theta)。
         * 精确 pi 处对数分支不光滑，只沿 poseErrorInBase 选定分支作局部尝试。
         */
        inline Eigen::Matrix3d rotationLogRightInverse(const Eigen::Vector3d &r)
        {
            const double theta2 = r.squaredNorm();
            const double theta = std::sqrt(theta2);
            const double beta = theta < 1e-3
                                    ? 1.0 / 12.0 + theta2 / 720.0 + theta2 * theta2 / 30240.0
                                    : (1.0 - 0.5 * theta / std::tan(0.5 * theta)) / theta2;
            Eigen::Matrix3d skew;
            skew << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
            return Eigen::Matrix3d::Identity() + 0.5 * skew + beta * skew * skew;
        }

        enum class BoxStatus
        {
            Solved,
            SweepLimit,
            TimeLimit,
            NumericalFailure
        };

        /**
         * solveNumericalIk 调用：解严格凸盒约束二次子问题
         *     min_z 0.5*z^T*H*z - g^T*z，lower <= z <= upper。
         * H 已含正阻尼。先用 LDLT 求无约束解作为起点，再投影并逐坐标最小化。
         * 不是“解一次再截断就结束”：截断后重新优化其他坐标，检查投影梯度映射。
         * z 为输出的归一化关节修正；只有 Solved 才可作为已收敛子问题步使用。
         * 所有循环有扫掠/时间预算，不把子问题没解完误报成整个 IK 成功。
         */
        template <int N>
        BoxStatus solveBoxStep(
            const Eigen::Matrix<double, N, N> &H,
            const Eigen::Matrix<double, N, 1> &g,
            const Eigen::Matrix<double, N, 1> &lower,
            const Eigen::Matrix<double, N, 1> &upper,
            const IkOptions &o, const Clock::time_point start,
            Eigen::Matrix<double, N, 1> &z)
        {
            using Vector = Eigen::Matrix<double, N, 1>;
            Eigen::LDLT<Eigen::Matrix<double, N, N>> factor{H};
            if (factor.info() != Eigen::Success || !factor.isPositive() ||
                !factor.vectorD().allFinite() || (factor.vectorD().array() <= 0.0).any())
                return BoxStatus::NumericalFailure;
            z = factor.solve(g);
            if (!z.allFinite())
                return BoxStatus::NumericalFailure;
            z = z.cwiseMax(lower).cwiseMin(upper);

            for (std::size_t sweep = 0;; ++sweep)
            {
                if (expired(start, o))
                    return BoxStatus::TimeLimit;
                const Vector gradient = H * z - g;
                if (!gradient.allFinite())
                    return BoxStatus::NumericalFailure;
                double residual = 0.0;
                for (int i = 0; i < N; ++i)
                {
                    const double projected = std::clamp(
                        z[i] - gradient[i] / H(i, i), lower[i], upper[i]);
                    residual = std::max(residual, std::abs(z[i] - projected));
                }
                if (residual <= o.subproblem_tolerance)
                    return BoxStatus::Solved;
                if (sweep >= o.max_coordinate_sweeps)
                    return BoxStatus::SweepLimit;
                for (int i = 0; i < N; ++i)
                {
                    const double derivative = H.row(i).dot(z) - g[i];
                    if (!std::isfinite(derivative))
                        return BoxStatus::NumericalFailure;
                    z[i] = std::clamp(z[i] - derivative / H(i, i), lower[i], upper[i]);
                }
            }
        }
    } // namespace numerical_ik_detail

    /**
     * 单目标、单初值、全位姿数值逆解：盒约束阻尼最小二乘 + 回溯验收。
     * 上游：初始化时创建 model/constraints；应用或路径采样器给 target 和 seed。
     * target 必须是 tip 相对 base 的目标，不能混用世界系或不同 TCP。
     * seed 是完整关节位置，不是增量；越界直接拒绝，不偷偷裁剪初值。seed是关节的初始位置
     * constraints 必须与模型/标定配套；本函数仅核对名称、类型与顺序。
     *
     * 每轮：FK -> e -> 雅可比及旋转误差导数修正 -> 有界修正量 -> FK 重算验收。
     * 令 A=[Jv; C*Jw]，则 e(q+dq)≈e-A*dq；不是直接把 J 当误差导数。
     * W 按位置/角度 scale 分别缩放；S 为各关节迭代步长尺度，dq=S*z。
     * B=W*A*S，r=W*e，每轮解：
     *     min_z 0.5*||r-B*z||^2 + 0.5*damping^2*||z||^2
     *     -1<=z<=1，且 lower_q<=q+S*z<=upper_q。
     * 用回溯和重算误差检查实际下降；被拒绝时增大阻尼再尝试。
     *
     * 成功只看原始位置/角度容差，同时保持位置限位；代价下降不是成功条件。
     * 不取模关节角，不随机重启、不保证全局可达性或最近解；NoProgress 不等于无解。
     * 不生成轨迹、不检查碰撞/速度/加速度、不驱动电机；成功也不能直接跳转下发。
     * 有预算检查，但单次 FK/分解不可被抢占，不能承诺严格硬实时截止。
     * 可预期输入/数学错误用结果状态返回；内存分配等系统异常不保证转换为状态。
     */
    template <std::size_t N>
    [[nodiscard]] IkResult<N> solveNumericalIk(
        const ValidatedUrdfChain<N> &model,
        const JointPositionConstraints<N> &constraints,
        const Pose &target, const std::array<double, N> &seed,
        const IkOptions &options = {})
    {
        static_assert(N > 0 && N <= static_cast<std::size_t>(std::numeric_limits<int>::max()));
        constexpr int D = static_cast<int>(N);
        using namespace numerical_ik_detail;
        using Vector = Eigen::Matrix<double, D, 1>;
        using Matrix = Eigen::Matrix<double, D, D>;
        using TaskVector = Eigen::Matrix<double, 6, 1>;
        using TaskJacobian = Eigen::Matrix<double, 6, D>;
        const auto start = Clock::now();
        IkResult<N> result;

        /** 各退出分支调用：统一记录原因、说明和已花时间。 */
        const auto finish = [&](IkStatus status, const std::string &message)
        {
            result.status = status;
            result.message = message;
            result.elapsed = Clock::now() - start;
            return result;
        };
        if (!validOptions(options))
            return finish(IkStatus::InvalidInput, "Invalid IK options");
        Pose goal;
        try
        {
            constraints.requireJointOrder(model);
            goal = normalizedPose(target);
        }
        catch (const std::invalid_argument &e)
        {
            return finish(IkStatus::InvalidInput, e.what());
        }
        for (double value : seed)
            if (!std::isfinite(value))
                return finish(IkStatus::InvalidInput, "Non-finite seed");
        if (!constraints.contains(seed))
            return finish(IkStatus::SeedOutOfBounds, "Seed violates position bounds");

        Vector step_scale = Vector::Constant(options.max_rotational_step);
        for (const auto &joint : model.chain().joints)
            if (joint.q_index)
                step_scale[static_cast<int>(*joint.q_index)] = joint.type == UrdfJointType::Prismatic
                                                                   ? options.max_prismatic_step
                                                                   : options.max_rotational_step;
        auto q = seed;
        double damping = options.initial_damping;
        TaskVector error, residual;
        double cost = 0.0;

        /** 入口/试步调用：FK 和原始误差由现有函数计算，再得到无量纲比较代价。 */
        const auto evaluate = [&](const std::array<double, N> &positions,
                                  TaskVector &e, TaskVector &r)
        {
            ++result.pose_evaluations;
            e = poseErrorInBase(forwardKinematics(model, positions), goal);
            r = e;
            r.head<3>() /= options.position_scale;
            r.tail<3>() /= options.rotation_scale;
            const double value = 0.5 * r.squaredNorm();
            if (!r.allFinite() || !std::isfinite(value))
                throw std::runtime_error("IK cost overflow");
            return value;
        };
        /** 仅在接受可行且数值有效的候选后保存诊断；不把它自动标记为成功。 */
        const auto saveCandidate = [&]
        {
            result.candidate_q = q;
            result.position_error = error.head<3>().stableNorm();
            result.rotation_error = error.tail<3>().stableNorm();
        };
        try
        {
            if (expired(start, options))
                return finish(IkStatus::TimeLimit, "IK time budget exhausted");
            cost = evaluate(q, error, residual);
            saveCandidate();
            for (;;)
            {
                if (expired(start, options))
                    return finish(IkStatus::TimeLimit, "IK time budget exhausted");
                if (result.position_error <= options.position_tolerance &&
                    result.rotation_error <= options.rotation_tolerance)
                    return finish(IkStatus::Converged, "Position and rotation tolerances satisfied");
                if (result.iterations >= options.max_iterations)
                    return finish(IkStatus::IterationLimit, "IK iteration budget exhausted");
                ++result.iterations;

                TaskJacobian A = geometricJacobian(model, q);
                const Eigen::Matrix3d C = rotationLogRightInverse(error.tail<3>());
                A.template bottomRows<3>() = (C * A.template bottomRows<3>()).eval();
                A.template topRows<3>() /= options.position_scale;
                A.template bottomRows<3>() /= options.rotation_scale;
                for (int i = 0; i < D; ++i)
                    A.col(i) *= step_scale[i];
                const Matrix normal = A.transpose() * A;
                const Vector gradient_rhs = A.transpose() * residual;
                if (!normal.allFinite() || !gradient_rhs.allFinite())
                    return finish(IkStatus::NumericalFailure, "IK linearization overflow");

                Vector lower = Vector::Constant(-1.0), upper = Vector::Constant(1.0);
                for (int i = 0; i < D; ++i)
                    if (const auto &bound = constraints.bounds()[static_cast<std::size_t>(i)])
                    {
                        lower[i] = std::max(-1.0, (bound->lower() - q[i]) / step_scale[i]);
                        upper[i] = std::min(1.0, (bound->upper() - q[i]) / step_scale[i]);
                    }
                bool accepted = false, solved_any = false, numerical_issue = false;
                for (std::size_t trial = 0; trial < options.max_damping_trials; ++trial)
                {
                    if (expired(start, options))
                        return finish(IkStatus::TimeLimit, "IK time budget exhausted");
                    Matrix H = normal;
                    H.diagonal().array() += damping * damping;
                    Vector z;
                    const auto box = solveBoxStep(H, gradient_rhs, lower, upper, options, start, z);
                    if (box == BoxStatus::TimeLimit)
                        return finish(IkStatus::TimeLimit, "Time budget exhausted in bounded step");
                    numerical_issue = numerical_issue || box == BoxStatus::NumericalFailure;
                    if (box == BoxStatus::Solved)
                    {
                        solved_any = true;
                        double alpha = 1.0;
                        for (std::size_t backtrack = 0; backtrack < options.max_backtracks; ++backtrack)
                        {
                            if (expired(start, options))
                                return finish(IkStatus::TimeLimit, "IK time budget exhausted");
                            auto candidate = q;
                            Vector actual_step;
                            for (int i = 0; i < D; ++i)
                            {
                                candidate[i] += alpha * step_scale[i] * z[i];
                                if (!std::isfinite(candidate[i]))
                                    throw std::runtime_error("IK candidate overflow");
                                // 子问题已满足区间；这里只消除 q+step 重建的浮点越界。
                                if (const auto &b = constraints.bounds()[static_cast<std::size_t>(i)])
                                    candidate[i] = std::clamp(candidate[i], b->lower(), b->upper());
                                actual_step[i] = (candidate[i] - q[i]) / step_scale[i];
                            }
                            const double descent = gradient_rhs.dot(actual_step);
                            if (std::isfinite(descent) && descent > 0.0 && constraints.contains(candidate))
                            {
                                TaskVector next_error, next_residual;
                                const double next_cost = evaluate(candidate, next_error, next_residual);
                                // 用真正重算的误差做 Armijo 下降验收，不能只看线性预测。
                                if (next_cost < cost && next_cost <= cost - 1e-4 * descent)
                                {
                                    q = candidate;
                                    error = next_error;
                                    residual = next_residual;
                                    cost = next_cost;
                                    saveCandidate();
                                    accepted = true;
                                    break;
                                }
                            }
                            alpha *= 0.5;
                        }
                    }
                    if (accepted)
                        break;
                    damping = std::min(1e6, damping * 10.0);
                }
                if (expired(start, options))
                    return finish(IkStatus::TimeLimit, "IK time budget exhausted");
                if (!accepted)
                {
                    if (solved_any)
                        return finish(IkStatus::NoProgress, "No acceptable descent step from this seed");
                    return finish(numerical_issue ? IkStatus::NumericalFailure : IkStatus::SubproblemLimit,
                                  "Bounded step could not be solved within its budget");
                }
                damping = std::max(1e-6, damping * 0.5);
            }
        }
        catch (const std::runtime_error &e)
        {
            return finish(IkStatus::NumericalFailure, e.what());
        }
        catch (const std::logic_error &e)
        {
            return finish(IkStatus::NumericalFailure, e.what());
        }
    }

} // namespace robot::motion