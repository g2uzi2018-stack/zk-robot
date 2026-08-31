#include "motion/planning/move_j.hpp"

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>

int main()
{
    using namespace robot::motion::planning;

    constexpr std::size_t N = 2;

    // =========================================================
    // 1. 起点 / 终点
    // =========================================================

    JointBoundaryState<N> start;
    start.position = {0.0, 0.0};
    start.velocity = {0.0, 0.0};
    start.acceleration = {0.0, 0.0};

    JointBoundaryState<N> goal;
    goal.position = {1.0, -0.8};
    goal.velocity = {0.0, 0.0};
    goal.acceleration = {0.0, 0.0};

    // =========================================================
    // 2. 关节限制
    // =========================================================

    JointMotionLimits<N> limits;

    limits.min_position = {-2.0, -2.0};
    limits.max_position = {2.0, 2.0};

    limits.max_velocity = {
        1.0,
        1.0};

    limits.max_acceleration = {
        2.0,
        2.0};

    limits.max_jerk = {
        10.0,
        10.0};

    // =========================================================
    // 3. MoveJ 时间搜索参数
    // =========================================================

    MoveJTimingOptions timing;

    timing.minimum_duration = std::chrono::milliseconds{500};
    timing.maximum_duration = std::chrono::seconds{5};
    timing.search_resolution = std::chrono::milliseconds{50};

    // =========================================================
    // 4. 调用完整版本 planMoveJ()
    // =========================================================

    MoveJDiagnostics diagnostics;

    try
    {
        const auto plan = planMoveJ(
            start,
            goal,
            limits,
            timing,
            &diagnostics);

        // =====================================================
        // 5. 打印规划结果
        // =====================================================

        const auto total_duration = plan.trajectory->duration();

        const double total_seconds =
            std::chrono::duration<double>(total_duration).count();

        std::cout << std::fixed << std::setprecision(6);

        std::cout << "====================================\n";
        std::cout << "MoveJ planning test\n";
        std::cout << "====================================\n";

        std::cout
            << "Search attempts   : "
            << diagnostics.search_attempts << '\n';

        std::cout
            << "Selected duration : "
            << std::chrono::duration<double>(
                   diagnostics.selected_duration)
                   .count()
            << " s\n";

        std::cout
            << "Search resolution : "
            << std::chrono::duration<double>(
                   diagnostics.search_resolution)
                   .count()
            << " s\n";

        std::cout
            << "Planning time     : "
            << diagnostics.planning_time_us
            << " us\n";

        // =====================================================
        // 6. 对规划结果进行多次 sample
        // =====================================================

        const std::array<int, 5> percentages{
            0, 25, 50, 75, 100};

        for (const int percent : percentages)
        {
            const auto elapsed =
                total_duration * percent / 100;

            const auto point =
                plan.trajectory->sample(elapsed);

            std::cout
                << "\n[" << percent << "%]"
                << "  t="
                << total_seconds *
                       static_cast<double>(percent) / 100.0
                << " s\n";

            for (std::size_t joint = 0;
                 joint < N;
                 ++joint)
            {
                std::cout
                    << "  joint " << joint
                    << ": position="
                    << point.position[joint]
                    << ", velocity="
                    << point.velocity[joint]
                    << ", acceleration="
                    << point.acceleration[joint]
                    << '\n';
            }

            std::cout
                << "  finished="
                << std::boolalpha
                << point.finished
                << '\n';
        }

        std::cout << "\n====================================\n";
        std::cout << "MoveJ test PASS\n";
        std::cout << "====================================\n";
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "MoveJ test FAILED: "
            << error.what()
            << '\n';

        return 1;
    }

    return 0;
}