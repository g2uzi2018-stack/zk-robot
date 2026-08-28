#include "motion/planning/move_j.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

void expect(const bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

template <typename Exception, typename Function>
void expectThrow(Function &&function, const std::string &message)
{
    try
    {
        function();
    }
    catch (const Exception &)
    {
        return;
    }
    throw std::runtime_error(message);
}

bool almostEqual(const double left, const double right,
                 const double tolerance = 1e-10)
{
    return std::abs(left - right) <= tolerance;
}

} // namespace

int main()
{
    try
    {
        using namespace std::chrono_literals;
        using robot::motion::planning::JointBoundaryState;
        using robot::motion::planning::JointMotionLimits;
        using robot::motion::planning::MoveJTimingOptions;
        using robot::motion::planning::planMoveJ;

        constexpr std::size_t kJointCount = 3;
        const std::array<double, kJointCount> start{0.0, -0.5, 0.25};
        const std::array<double, kJointCount> goal{1.0, -1.0, -0.25};
        const JointMotionLimits<kJointCount> limits{
            {-2.0, -2.0, -2.0},
            {2.0, 2.0, 2.0},
            {0.6, 0.5, 0.4},
            {1.0, 0.8, 0.7},
            {4.0, 3.0, 2.0}};

        const auto plan = planMoveJ(start, goal, limits, 10ms);
        expect(static_cast<bool>(plan.trajectory),
               "MoveJ did not return a trajectory");
        expect(plan.velocity_limits == limits.max_velocity,
               "MoveJ execution velocity limits changed");

        const auto at_start = plan.trajectory->sample(
            robot::motion::JointTrajectory<kJointCount>::Duration::zero());
        expect(at_start.position == start && !at_start.finished,
               "MoveJ start sample mismatch");

        const auto at_end = plan.trajectory->sample(plan.trajectory->duration());
        expect(at_end.finished, "MoveJ end sample was not marked finished");
        for (std::size_t joint = 0; joint < kJointCount; ++joint)
        {
            expect(almostEqual(at_end.position[joint], goal[joint]),
                   "MoveJ did not end at the requested goal");
            expect(almostEqual(at_end.velocity[joint], 0.0),
                   "MoveJ end velocity is not zero");
            expect(almostEqual(at_end.acceleration[joint], 0.0),
                   "MoveJ end acceleration is not zero");
        }

        // 密集采样验证生成轨迹的速度和加速度没有越过规划约束。Jerk 的
        // 精确峰值发生在端点，直接使用五次时间缩放的解析式验证。
        constexpr int kSamples = 2000;
        const double duration_seconds = std::chrono::duration<double>(
            plan.trajectory->duration()).count();
        for (int sample_index = 0; sample_index <= kSamples; ++sample_index)
        {
            const double ratio =
                static_cast<double>(sample_index) / kSamples;
            const auto elapsed = std::chrono::duration_cast<
                robot::motion::JointTrajectory<kJointCount>::Duration>(
                std::chrono::duration<double>(duration_seconds * ratio));
            const auto point = plan.trajectory->sample(elapsed);

            for (std::size_t joint = 0; joint < kJointCount; ++joint)
            {
                expect(std::abs(point.velocity[joint]) <=
                           limits.max_velocity[joint] + 1e-9,
                       "MoveJ exceeded a velocity limit");
                expect(std::abs(point.acceleration[joint]) <=
                           limits.max_acceleration[joint] + 1e-9,
                       "MoveJ exceeded an acceleration limit");
            }
        }

        for (std::size_t joint = 0; joint < kJointCount; ++joint)
        {
            const double displacement = std::abs(goal[joint] - start[joint]);
            const double peak_jerk =
                60.0 * displacement /
                (duration_seconds * duration_seconds * duration_seconds);
            expect(peak_jerk <= limits.max_jerk[joint] + 1e-9,
                   "MoveJ exceeded a jerk limit");
        }

        // 完整入口允许起点和终点具有非零速度、加速度。规划器需要在给定
        // 时间范围内找到一条连续满足全部约束的五次轨迹。
        const JointBoundaryState<kJointCount> moving_start{
            start,
            {0.10, -0.05, 0.02},
            {0.02, 0.00, -0.01}};
        const JointBoundaryState<kJointCount> moving_goal{
            goal,
            {0.05, 0.02, -0.03},
            {-0.02, 0.01, 0.00}};
        const MoveJTimingOptions timing{100ms, 6s, 5ms};

        const auto moving_plan = planMoveJ(
            moving_start, moving_goal, limits, timing);
        const auto moving_at_start = moving_plan.trajectory->sample(
            robot::motion::JointTrajectory<kJointCount>::Duration::zero());
        const auto moving_at_end = moving_plan.trajectory->sample(
            moving_plan.trajectory->duration());

        for (std::size_t joint = 0; joint < kJointCount; ++joint)
        {
            expect(almostEqual(moving_at_start.position[joint],
                               moving_start.position[joint]) &&
                       almostEqual(moving_at_start.velocity[joint],
                                   moving_start.velocity[joint]) &&
                       almostEqual(moving_at_start.acceleration[joint],
                                   moving_start.acceleration[joint]),
                   "MoveJ non-zero start boundary mismatch");
            expect(almostEqual(moving_at_end.position[joint],
                               moving_goal.position[joint]) &&
                       almostEqual(moving_at_end.velocity[joint],
                                   moving_goal.velocity[joint]) &&
                       almostEqual(moving_at_end.acceleration[joint],
                                   moving_goal.acceleration[joint]),
                   "MoveJ non-zero goal boundary mismatch");
        }

        const double moving_duration_seconds =
            std::chrono::duration<double>(
                moving_plan.trajectory->duration()).count();
        for (int sample_index = 0; sample_index <= kSamples; ++sample_index)
        {
            const double ratio =
                static_cast<double>(sample_index) / kSamples;
            const auto elapsed = std::chrono::duration_cast<
                robot::motion::JointTrajectory<kJointCount>::Duration>(
                std::chrono::duration<double>(
                    moving_duration_seconds * ratio));
            const auto point = moving_plan.trajectory->sample(elapsed);

            for (std::size_t joint = 0; joint < kJointCount; ++joint)
            {
                expect(point.position[joint] >=
                           limits.min_position[joint] - 1e-9 &&
                           point.position[joint] <=
                               limits.max_position[joint] + 1e-9,
                       "MoveJ non-zero trajectory exceeded a position limit");
                expect(std::abs(point.velocity[joint]) <=
                           limits.max_velocity[joint] + 1e-9,
                       "MoveJ non-zero trajectory exceeded a velocity limit");
                expect(std::abs(point.acceleration[joint]) <=
                           limits.max_acceleration[joint] + 1e-9,
                       "MoveJ non-zero trajectory exceeded an acceleration limit");
            }
        }

        // 边界状态本身超过限制时应立即拒绝，而不是进入 duration 搜索。
        auto excessive_velocity_start = moving_start;
        excessive_velocity_start.velocity[0] = 0.7;
        expectThrow<std::out_of_range>(
            [&]() {
                planMoveJ(excessive_velocity_start, moving_goal,
                          limits, timing);
            },
            "MoveJ accepted an excessive boundary velocity");

        // 时间范围过短时没有可行轨迹，规划必须明确失败。
        const MoveJTimingOptions insufficient_time{10ms, 100ms, 5ms};
        expectThrow<std::runtime_error>(
            [&]() {
                planMoveJ(moving_start, moving_goal,
                          limits, insufficient_time);
            },
            "MoveJ accepted an infeasible duration range");

        // 零位移仍返回由 minimum_duration 定义的合法保持轨迹，而不是构造
        // duration=0 的 QuinticJointTrajectory。
        const auto hold_plan = planMoveJ(start, start, limits, 25ms);
        expect(hold_plan.trajectory->duration() >= 25ms,
               "MoveJ did not preserve minimum duration");
        expect(hold_plan.trajectory->sample(
                   hold_plan.trajectory->duration()).position == start,
               "zero-displacement MoveJ changed joint positions");

        auto invalid_limits = limits;
        invalid_limits.max_velocity[1] = 0.0;
        expectThrow<std::invalid_argument>(
            [&]() { planMoveJ(start, goal, invalid_limits, 10ms); },
            "MoveJ accepted a zero velocity limit");

        auto invalid_goal = goal;
        invalid_goal[0] = std::numeric_limits<double>::quiet_NaN();
        expectThrow<std::invalid_argument>(
            [&]() { planMoveJ(start, invalid_goal, limits, 10ms); },
            "MoveJ accepted a non-finite goal");

        expectThrow<std::invalid_argument>(
            [&]() { planMoveJ(start, goal, limits, 0ms); },
            "MoveJ accepted a non-positive minimum duration");

        std::cout << "Motion MoveJ tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Motion MoveJ test failed: " << error.what() << '\n';
        return 1;
    }
}
