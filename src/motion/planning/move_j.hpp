#pragma once

#include "motion/trajectory/joint_trajectory.hpp"
#include "motion/trajectory/quintic_joint_trajectory.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace robot::motion::planning
{

    using MotionClock = std::chrono::steady_clock;
    using MotionDuration = MotionClock::duration;

    // MoveJ 的一个完整关节边界状态。
    //
    // position / velocity / acceleration 都位于机器人模型关节坐标中：
    //
    //   - 旋转关节使用 rad、rad/s、rad/s^2；
    //   - 移动关节使用 m、m/s、m/s^2。
    //
    // 编码器 counts、motor_rad、方向和模型零偏必须在 Joint 层完成转换，不能进入
    // 规划接口。非零 velocity / acceleration 使相邻运动可以连续衔接，也允许应用
    // 使用机器人当前真实运动状态作为重新规划的起点。
    template <std::size_t N>
    struct JointBoundaryState
    {
        std::array<double, N> position{};
        std::array<double, N> velocity{};
        std::array<double, N> acceleration{};
    };

    // 一组关节的完整 MoveJ 约束。
    //
    // 非零边界速度可能使五次曲线短暂越过起点和终点，因此除了动态约束，还必须
    // 检查整条轨迹的模型关节位置范围，不能只依赖 Executor 在下发时被动报错。
    template <std::size_t N>
    struct JointMotionLimits
    {
        std::array<double, N> min_position{};
        std::array<double, N> max_position{};
        std::array<double, N> max_velocity{};
        std::array<double, N> max_acceleration{};
        std::array<double, N> max_jerk{}; // 加速度的导数,描述加速度的变化趋势
    };

    // 非零边界状态下的持续时间搜索约束。
    //
    // 对任意起终速度和加速度，“duration 越长越安全”并不总成立。例如较大的非零
    // 边界加速度在很长时间内反而可能造成更大的中间速度和位置过冲。因此规划器在
    // 一个明确、有界的区间内逐个检查候选 duration，而不是无限增加时间。
    //
    // search_resolution 通常取 Executor 控制周期。这样选出的 duration 具有清晰的
    // 系统时间粒度，同时保证搜索工作量有界。
    //
    // duration 指：整条 MoveJ 轨迹从开始到结束，总共要花多长时间

    struct MoveJTimingOptions
    {
        MotionDuration minimum_duration{};
        MotionDuration maximum_duration{};
        MotionDuration search_resolution{}; // 规划阶段搜索 duration 的步长
    };

    // planMoveJ() 的规划结果。
    //
    // trajectory:
    //   已确定边界状态和持续时间，可由 Executor 按控制周期 sample()。
    //
    // velocity_limits:
    //   直接交给当前 Controller / Executor 接口的执行速度限制。它与规划使用的
    //   max_velocity 相同，避免规划约束和底层命令约束使用两套数据。
    template <std::size_t N>
    struct PlannedJointMotion
    {
        std::shared_ptr<const JointTrajectory<N>> trajectory;
        std::array<double, N> velocity_limits{};
    };

    // 统计参数
    struct MoveJDiagnostics
    {
        std::size_t search_attempts{0};
        MotionDuration selected_duration{};
        MotionDuration search_resolution{};
        double planning_time_us{0.0};
    };

    namespace detail
    {

        // 约定coefficients[i] = x^i 的系数
        using Polynomial = std::vector<double>;

        inline void validatePositiveFinite(const double value, const char *message)
        {
            if (!std::isfinite(value) || value <= 0.0)
            {
                throw std::invalid_argument(message);
            }
        }

        inline double evaluatePolynomial(const Polynomial &coefficients, const double value)
        {
            // Horner 形式从最高次项向常数项求值，减少乘法次数和舍入误差。
            double result = 0.0;
            for (auto coefficient = coefficients.rbegin(); coefficient != coefficients.rend(); ++coefficient)
            {
                result = result * value + *coefficient;
            }
            return result;
        }

        // 对多项式求导 返回导数的多项式
        inline Polynomial derivative(const Polynomial &coefficients)
        {
            if (coefficients.size() <= 1)
            {
                return Polynomial{0.0};
            }

            Polynomial result(coefficients.size() - 1);
            for (std::size_t power = 1; power < coefficients.size(); ++power)
            {
                result[power - 1] =
                    static_cast<double>(power) * coefficients[power];
            }
            return result;
        }

        // 找最多项式最大的系数的绝对值 用于计算多项式的数值尺度
        inline double polynomialScale(const Polynomial &coefficients)
        {
            double scale = 1.0;
            for (const double coefficient : coefficients)
            {
                scale = std::max(scale, std::abs(coefficient));
            }
            return scale;
        }

        // 移除最高次项的系数为零(接近)的项，避免在求根时产生不必要的数值问题。
        inline void trimLeadingZeros(Polynomial &coefficients)
        {
            const double tolerance = 1e-13 * polynomialScale(coefficients);
            while (coefficients.size() > 1 &&
                   std::abs(coefficients.back()) <= tolerance)
            {
                coefficients.pop_back();
            }
        }

        inline void addUniqueRoot(std::vector<double> &roots, double root)
        {
            constexpr double kRootTolerance = 1e-10;

            if (root < -kRootTolerance || root > 1.0 + kRootTolerance)
            {
                return;
            }

            root = std::clamp(root, 0.0, 1.0);
            const auto duplicate = std::find_if(
                roots.begin(), roots.end(),
                [root](const double existing)
                {
                    return std::abs(existing - root) <= kRootTolerance;
                });
            if (duplicate == roots.end())
            {
                roots.push_back(root);
            }
        }

        // 返回多项式在闭区间 [0, 1] 内的全部实根。
        //
        // 这里只需要处理最高四次的导数多项式。实现采用递归导数隔离：先求导数的根，
        // 把区间分成若干单调段，再在发生符号变化的单调段内二分。这比在固定采样点上
        // 猜测峰值可靠，也能通过导数根识别偶重根。
        inline std::vector<double> rootsInUnitInterval(Polynomial coefficients)
        {
            trimLeadingZeros(coefficients);

            const std::size_t degree = coefficients.size() - 1;
            if (degree == 0)
            {
                // 恒零多项式意味着区间内处处为根。对于极值查找无需返回无限多个点，
                // 后续检查区间端点已经足够。
                return {};
            }

            if (degree == 1)
            {
                std::vector<double> roots;
                addUniqueRoot(roots, -coefficients[0] / coefficients[1]);
                return roots;
            }

            auto critical_points = rootsInUnitInterval(derivative(coefficients));
            critical_points.push_back(0.0);
            critical_points.push_back(1.0);
            std::sort(critical_points.begin(), critical_points.end());
            critical_points.erase(
                std::unique(
                    critical_points.begin(), critical_points.end(),
                    [](const double left, const double right)
                    {
                        return std::abs(left - right) <= 1e-12;
                    }),
                critical_points.end());

            const double value_tolerance =
                1e-11 * polynomialScale(coefficients);
            std::vector<double> roots;

            // 偶重根不会产生符号变化，但它一定也是导数根，因此先检查所有分段点。
            for (const double point : critical_points)
            {
                if (std::abs(evaluatePolynomial(coefficients, point)) <=
                    value_tolerance)
                {
                    addUniqueRoot(roots, point);
                }
            }

            for (std::size_t interval = 0;
                 interval + 1 < critical_points.size(); ++interval)
            {
                double left = critical_points[interval];
                double right = critical_points[interval + 1];
                double left_value = evaluatePolynomial(coefficients, left);
                const double right_value = evaluatePolynomial(coefficients, right);

                if (!((left_value < 0.0 && right_value > 0.0) ||
                      (left_value > 0.0 && right_value < 0.0)))
                {
                    continue;
                }

                // 每个区间在导数根之间单调，因此二分只可能收敛到一个根。
                for (int iteration = 0; iteration < 80; ++iteration)
                {
                    const double middle = 0.5 * (left + right);
                    const double middle_value =
                        evaluatePolynomial(coefficients, middle);

                    if (std::abs(middle_value) <= value_tolerance)
                    {
                        left = middle;
                        right = middle;
                        break;
                    }

                    if ((left_value < 0.0 && middle_value > 0.0) ||
                        (left_value > 0.0 && middle_value < 0.0))
                    {
                        right = middle;
                    }
                    else
                    {
                        left = middle;
                        left_value = middle_value;
                    }
                }

                addUniqueRoot(roots, 0.5 * (left + right));
            }

            std::sort(roots.begin(), roots.end());
            return roots;
        }

        // 最小值、最大值、最大绝对值
        struct ValueRange
        {
            double minimum{0.0};
            double maximum{0.0};
            double maximum_absolute{0.0};
        };

        // 求一个多项式在 [0,1] 内的连续极值，而不是只检查离散采样点。
        inline ValueRange polynomialRange(const Polynomial &coefficients, const double physical_scale)
        {
            const auto consider = [physical_scale](ValueRange &range, const double raw_value)
            {
                const double value = raw_value * physical_scale;
                range.minimum = std::min(range.minimum, value);
                range.maximum = std::max(range.maximum, value);
                range.maximum_absolute =
                    std::max(range.maximum_absolute, std::abs(value));
            };

            const double initial =
                evaluatePolynomial(coefficients, 0.0) * physical_scale;
            ValueRange result{initial, initial, std::abs(initial)};

            consider(result, evaluatePolynomial(coefficients, 1.0));
            for (const double root :
                 rootsInUnitInterval(derivative(coefficients)))
            {
                consider(result, evaluatePolynomial(coefficients, root));
            }
            return result;
        }

        // 根据起点、终点的位置/速度/加速度和总时间 T，生成那条五次轨迹的系数。这个是比较核心的
        //  在归一化时间 u=t/T 中构造五次多项式。归一化后根搜索始终发生在 [0,1]，
        //  避免直接在纳秒或很大的秒数上求根造成不必要的数值尺度问题。
        /**
         * @param start_position 起点位置
         * @param start_velocity 起点速度
         * @param start_acceleration 起点加速度
         * @param goal_position 终点位置
         * @param goal_velocity 终点速度
         * @param goal_acceleration 终点加速度
         * @param duration_seconds 轨迹总时间
         * @return 五次多项式
         */
        inline Polynomial normalizedQuinticCoefficients(const double start_position, const double start_velocity, const double start_acceleration, const double goal_position, const double goal_velocity, const double goal_acceleration, const double duration_seconds)
        {

            // 五次多项式系数推导：
            //
            // 归一化时间：
            //   u = t / T,  u ∈ [0, 1]
            //
            // 设关节轨迹为：
            //   q(u) = a0 + a1*u + a2*u^2 + a3*u^3 + a4*u^4 + a5*u^5
            //
            // 对 u 求导：
            //   dq/du   = a1 + 2*a2*u + 3*a3*u^2 + 4*a4*u^3 + 5*a5*u^4
            //   d2q/du2 = 2*a2 + 6*a3*u + 12*a4*u^2 + 20*a5*u^3
            //
            // 因为 u = t / T，所以真实速度、加速度为：
            //   v(t)   = dq/dt   = (1/T)   * dq/du
            //   acc(t) = d2q/dt2 = (1/T^2) * d2q/du2
            //
            // 已知 6 个边界条件：
            //   q(0)   = start_position
            //   v(0)   = start_velocity
            //   acc(0) = start_acceleration
            //
            //   q(1)   = goal_position
            //   v(1)   = goal_velocity
            //   acc(1) = goal_acceleration
            //
            // 代入起点 u = 0：
            //   a0 = start_position
            //   a1 = start_velocity * T
            //   a2 = 0.5 * start_acceleration * T^2
            //
            // 再将 u = 1 代入终点的三个边界条件，得到关于 a3、a4、a5 的
            // 三元一次方程组，联立求解后得到：
            //
            //   a3 = 10*Δq
            //        - (6*v0 + 4*v1)*T
            //        - (1.5*acc0 - 0.5*acc1)*T^2
            //
            //   a4 = -15*Δq
            //        + (8*v0 + 7*v1)*T
            //        + (1.5*acc0 - acc1)*T^2
            //
            //   a5 = 6*Δq
            //        - 3*(v0 + v1)*T
            //        - 0.5*(acc0 - acc1)*T^2
            //
            // 其中：
            //   Δq   = goal_position - start_position
            //   v0   = start_velocity
            //   v1   = goal_velocity
            //   acc0 = start_acceleration
            //   acc1 = goal_acceleration
            //
            // 因此，6 个边界条件唯一确定五次多项式的 6 个系数 a0 ~ a5。

            // 轨迹总时间
            const double T = duration_seconds;
            const double T2 = T * T;

            // 关节总位移
            const double displacement = goal_position - start_position;

            // 五次多项式：
            // q(u) = a0 + a1*u + a2*u^2 + a3*u^3 + a4*u^4 + a5*u^5
            // 其中归一化时间 u = t / T，u ∈ [0, 1]

            // 起点的位置、速度、加速度条件直接确定前三个系数
            const double a0 = start_position;
            const double a1 = start_velocity * T;
            const double a2 = 0.5 * start_acceleration * T2;

            // 剩余三个系数由终点的位置、速度、加速度条件联立求解得到
            const double a3 = 10.0 * displacement - (6.0 * start_velocity + 4.0 * goal_velocity) * T - (1.5 * start_acceleration - 0.5 * goal_acceleration) * T2;

            const double a4 = -15.0 * displacement + (8.0 * start_velocity + 7.0 * goal_velocity) * T + (1.5 * start_acceleration - goal_acceleration) * T2;

            const double a5 = 6.0 * displacement - 3.0 * (start_velocity + goal_velocity) * T - 0.5 * (start_acceleration - goal_acceleration) * T2;

            return Polynomial{a0, a1, a2, a3, a4, a5};
        }

        // 判断某个值有没有超过限制，同时留一点浮点误差余量
        inline bool withinLimit(const double value, const double limit)
        {
            const double tolerance = 1e-9 * std::max(1.0, std::abs(limit));
            return value <= limit + tolerance;
        }

        // 检查 MoveJ 输入是否合法，比如位置、速度、加速度、各种限制; 尽管输入的机械臂状态大概率合法,但是实际上目标状态可能不合法,所以需要检查
        template <std::size_t N>
        void validateRequest(const JointBoundaryState<N> &start, const JointBoundaryState<N> &goal, const JointMotionLimits<N> &limits)
        {
            static_assert(N > 0, "MoveJ requires at least one joint");

            for (std::size_t joint = 0; joint < N; ++joint)
            {
                const std::array<double, 6> boundary_values{
                    start.position[joint], start.velocity[joint],
                    start.acceleration[joint], goal.position[joint],
                    goal.velocity[joint], goal.acceleration[joint]};
                for (const double value : boundary_values)
                {
                    if (!std::isfinite(value))
                    {
                        throw std::invalid_argument(
                            "MoveJ boundary states must be finite");
                    }
                }

                if (!std::isfinite(limits.min_position[joint]) ||
                    !std::isfinite(limits.max_position[joint]) ||
                    limits.min_position[joint] >= limits.max_position[joint])
                {
                    throw std::invalid_argument(
                        "MoveJ position limits must be finite and increasing");
                }

                validatePositiveFinite(
                    limits.max_velocity[joint],
                    "MoveJ maximum velocity must be positive and finite");
                validatePositiveFinite(
                    limits.max_acceleration[joint],
                    "MoveJ maximum acceleration must be positive and finite");
                validatePositiveFinite(
                    limits.max_jerk[joint],
                    "MoveJ maximum jerk must be positive and finite");

                if (start.position[joint] < limits.min_position[joint] ||
                    start.position[joint] > limits.max_position[joint] ||
                    goal.position[joint] < limits.min_position[joint] ||
                    goal.position[joint] > limits.max_position[joint])
                {
                    throw std::out_of_range(
                        "MoveJ boundary position exceeds a joint limit");
                }

                if (!withinLimit(std::abs(start.velocity[joint]),
                                 limits.max_velocity[joint]) ||
                    !withinLimit(std::abs(goal.velocity[joint]),
                                 limits.max_velocity[joint]))
                {
                    throw std::out_of_range(
                        "MoveJ boundary velocity exceeds a joint limit");
                }

                if (!withinLimit(std::abs(start.acceleration[joint]),
                                 limits.max_acceleration[joint]) ||
                    !withinLimit(std::abs(goal.acceleration[joint]),
                                 limits.max_acceleration[joint]))
                {
                    throw std::out_of_range(
                        "MoveJ boundary acceleration exceeds a joint limit");
                }
            }
        }

        // 检查 minimum_duration / maximum_duration / search_resolution 最大最小搜索步长是否合理
        inline void validateTiming(const MoveJTimingOptions &timing)
        {
            if (timing.minimum_duration <= MotionDuration::zero() ||
                timing.maximum_duration < timing.minimum_duration ||
                timing.search_resolution <= MotionDuration::zero())
            {
                throw std::invalid_argument(
                    "MoveJ timing range and search resolution are invalid");
            }

            const long double range_seconds =
                std::chrono::duration<long double>(
                    timing.maximum_duration - timing.minimum_duration)
                    .count();
            const long double resolution_seconds =
                std::chrono::duration<long double>(
                    timing.search_resolution)
                    .count();

            // 防止误把纳秒作为搜索步长并在几十秒范围内执行数十亿次检查。
            constexpr long double kMaximumSearchSteps = 1000000.0L;
            if (range_seconds / resolution_seconds > kMaximumSearchSteps)
            {
                throw std::invalid_argument(
                    "MoveJ timing range contains too many search steps");
            }
        }

        // 真正检查整条轨迹有没有超过位置、速度、加速度、jerk 限制。这个也是核心
        template <std::size_t N>
        bool trajectorySatisfiesLimits(const JointBoundaryState<N> &start, const JointBoundaryState<N> &goal, const JointMotionLimits<N> &limits, const MotionDuration duration)
        {
            const double T = std::chrono::duration<double>(duration).count();
            if (!std::isfinite(T) || T <= 0.0)
            {
                return false;
            }

            // 倒数
            const double inverse_T = 1.0 / T;
            const double inverse_T2 = inverse_T * inverse_T;
            const double inverse_T3 = inverse_T2 * inverse_T;

            for (std::size_t joint = 0; joint < N; ++joint)
            {
                // 根据 start / goal / T 生成 position 五次多项式
                const Polynomial position = normalizedQuinticCoefficients(start.position[joint], start.velocity[joint],
                                                                          start.acceleration[joint], goal.position[joint],
                                                                          goal.velocity[joint], goal.acceleration[joint], T);

                /**
                 * 连续求导得到 velocity / acceleration / jerk
                 * */
                const Polynomial velocity = derivative(position);
                const Polynomial acceleration = derivative(velocity);
                const Polynomial jerk = derivative(acceleration);

                // 求它们在整段 u∈[0,1] 里的最大最小值
                const ValueRange position_range =
                    polynomialRange(position, 1.0);
                const ValueRange velocity_range =
                    polynomialRange(velocity, inverse_T);
                const ValueRange acceleration_range =
                    polynomialRange(acceleration, inverse_T2);
                const ValueRange jerk_range =
                    polynomialRange(jerk, inverse_T3);

                const double position_tolerance = 1e-9 * std::max(
                                                             {1.0, std::abs(limits.min_position[joint]),
                                                              std::abs(limits.max_position[joint])});
                // 和 joint limits 比较
                if (position_range.minimum <
                        limits.min_position[joint] - position_tolerance ||
                    position_range.maximum >
                        limits.max_position[joint] + position_tolerance ||
                    !withinLimit(velocity_range.maximum_absolute,
                                 limits.max_velocity[joint]) ||
                    !withinLimit(acceleration_range.maximum_absolute,
                                 limits.max_acceleration[joint]) ||
                    !withinLimit(jerk_range.maximum_absolute,
                                 limits.max_jerk[joint]))
                {
                    return false;
                }
            }

            return true;
        }

        // 创建真正的 五次关节轨迹，再包装成 PlannedJointMotion
        template <std::size_t N>
        PlannedJointMotion<N> makePlan(const JointBoundaryState<N> &start,
                                       const JointBoundaryState<N> &goal,
                                       const JointMotionLimits<N> &limits,
                                       const MotionDuration duration)
        {
            using Trajectory = QuinticJointTrajectory<N>;

            const typename Trajectory::BoundaryState trajectory_start{
                start.position, start.velocity, start.acceleration};
            const typename Trajectory::BoundaryState trajectory_goal{
                goal.position, goal.velocity, goal.acceleration};

            PlannedJointMotion<N> result;
            result.trajectory = std::make_shared<const Trajectory>(
                trajectory_start, trajectory_goal, duration);
            result.velocity_limits = limits.max_velocity;
            return result;
        }

        // 浮点秒数安全地转成 MotionDuration，并向上取，避免时间被舍短
        inline MotionDuration ceilDuration(const double seconds)
        {
            if (!std::isfinite(seconds) || seconds <= 0.0)
            {
                throw std::overflow_error("MoveJ duration is not finite");
            }

            const double maximum_seconds =
                std::chrono::duration<double>(MotionDuration::max()).count();
            if (seconds > maximum_seconds)
            {
                throw std::overflow_error("MoveJ duration exceeds clock range");
            }

            return std::chrono::ceil<MotionDuration>(
                std::chrono::duration<double>(seconds));
        }

    } // namespace detail

    // 完整 MoveJ 入口：支持非零起终速度和加速度。
    //
    // 上游：
    //   应用从 Executor 的状态快照取得 start，提供目标 boundary state、机器人模型
    //   约束和有界时间搜索配置。规划器不主动读取物理世界。
    //
    // 下游：
    //   返回的 trajectory 和 velocity_limits 可以直接提交给对应机器人的 Executor。
    //
    // 本函数不创建线程。它按照 search_resolution 从 minimum_duration 向
    // maximum_duration 检查候选，并返回第一个满足连续位置、速度、加速度和 jerk
    // 约束的五次轨迹，即该时间分辨率下的最短可行结果。
    template <std::size_t N>
    PlannedJointMotion<N> planMoveJ(
        const JointBoundaryState<N> &start,
        const JointBoundaryState<N> &goal,
        const JointMotionLimits<N> &limits,
        const MoveJTimingOptions &timing,
        MoveJDiagnostics *diagnostics = nullptr)
    {
        // 只有调用者需要诊断数据时才记录规划开始时间
        MotionClock::time_point planning_begin{};
        if (diagnostics)
        {
            planning_begin = MotionClock::now();
            *diagnostics = {};
            diagnostics->search_resolution = timing.search_resolution;
        }

        // 检查输入和时间搜索参数
        detail::validateRequest(start, goal, limits);
        detail::validateTiming(timing);

        std::size_t attempts = 0;

        // 从 minimum_duration 开始搜索候选总时间 T
        MotionDuration candidate = timing.minimum_duration;

        while (true)
        {
            // 每检查一个候选 T，记为一次尝试
            ++attempts;

            if (detail::trajectorySatisfiesLimits(
                    start, goal, limits, candidate))
            {
                // 成功后先保存诊断数据，再返回轨迹
                if (diagnostics)
                {
                    diagnostics->search_attempts = attempts;
                    diagnostics->selected_duration = candidate;

                    diagnostics->planning_time_us =
                        std::chrono::duration<double, std::micro>(
                            MotionClock::now() - planning_begin)
                            .count();
                }

                return detail::makePlan(
                    start, goal, limits, candidate);
            }

            if (candidate == timing.maximum_duration)
            {
                break;
            }

            const MotionDuration remaining =
                timing.maximum_duration - candidate;

            candidate +=
                remaining <= timing.search_resolution
                    ? remaining
                    : timing.search_resolution;
        }

        // 搜索失败也保留诊断信息
        if (diagnostics)
        {
            diagnostics->search_attempts = attempts;

            diagnostics->planning_time_us =
                std::chrono::duration<double, std::micro>(
                    MotionClock::now() - planning_begin)
                    .count();
        }

        throw std::runtime_error(
            "MoveJ has no feasible quintic trajectory in the requested time range");
    }

    // 静止到静止的便利入口。
    //
    // 这是完整边界状态入口的特例，不是另一套规划语义。标准五次时间缩放的速度、
    // 加速度和 jerk 峰值具有闭式解，因此可以直接计算满足约束的共同 duration，
    // 无需执行离散时间搜索。
    template <std::size_t N>
    PlannedJointMotion<N> planMoveJ(const std::array<double, N> &start_position, const std::array<double, N> &goal_position, const JointMotionLimits<N> &limits, const MotionDuration minimum_duration)
    {
        const JointBoundaryState<N> start{start_position, {}, {}};
        const JointBoundaryState<N> goal{goal_position, {}, {}};
        detail::validateRequest(start, goal, limits);

        if (minimum_duration <= MotionDuration::zero())
        {
            throw std::invalid_argument(
                "MoveJ minimum duration must be positive");
        }

        double required_seconds =
            std::chrono::duration<double>(minimum_duration).count();

        constexpr double kPeakVelocityFactor = 15.0 / 8.0;
        const double peak_acceleration_factor = 10.0 / std::sqrt(3.0);
        constexpr double kPeakJerkFactor = 60.0;

        for (std::size_t joint = 0; joint < N; ++joint)
        {
            const double displacement =
                std::abs(goal_position[joint] - start_position[joint]);

            required_seconds = std::max(
                {required_seconds,
                 kPeakVelocityFactor * displacement /
                     limits.max_velocity[joint],
                 std::sqrt(peak_acceleration_factor * displacement /
                           limits.max_acceleration[joint]),
                 std::cbrt(kPeakJerkFactor * displacement /
                           limits.max_jerk[joint])});
        }

        const MotionDuration duration = detail::ceilDuration(required_seconds);
        if (!detail::trajectorySatisfiesLimits(
                start, goal, limits, duration))
        {
            // 对静止到静止的标准五次曲线，闭式峰值计算应当与连续极值检查一致。
            // 进入这里表示数值或实现错误，不能把未验证的轨迹交给 Executor。
            throw std::runtime_error(
                "MoveJ analytical duration failed continuous limit validation");
        }

        return detail::makePlan(start, goal, limits, duration);
    }

} // namespace robot::motion::planning
