#include "motion/kinematics/urdf_chain.hpp"
#include "motion/planning/cartesian_ik_path.hpp"
#include "motion/planning/joint_acceleration_limits.hpp"
#include "motion/planning/velocity_time_parameterization.hpp"
#include "motion/trajectory/cubic_joint_trajectory.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

    using namespace robot::motion;
    using namespace robot::motion::planning;

    constexpr std::size_t kJointCount = 7;

    using JointVector =
        std::array<double, kJointCount>;

    using Trajectory =
        CubicJointTrajectory<kJointCount>;

    using Duration =
        JointTrajectory<kJointCount>::Duration;

    /*
     * 测试动态参数。
     *
     * acceleration 仍然只是测试值，
     * 不是T170C真实参数。
     */
    struct DynamicConfig
    {
        const char *name;
        double velocity;
        double acceleration;
    };

    constexpr std::array<
        DynamicConfig,
        kJointCount>
        kDynamics{{
            {"L_SHOULDER_P", 7.12, 40.0},
            {"L_SHOULDER_R", 7.12, 40.0},
            {"L_SHOULDER_Y", 10.16, 40.0},
            {"L_ELBOW_Y", 10.16, 40.0},
            {"L_WRIST_P", 8.38, 40.0},
            {"L_WRIST_Y", 9.42, 40.0},
            {"L_WRIST_R", 9.42, 40.0},
        }};

    void require(
        bool condition,
        const std::string &message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    std::vector<NamedJointVelocityLimit>
    velocityLimits()
    {
        std::vector<NamedJointVelocityLimit> result;

        for (const auto &item : kDynamics)
        {
            result.push_back(
                {item.name,
                 item.velocity});
        }

        return result;
    }

    std::vector<NamedJointAccelerationLimit>
    accelerationLimits()
    {
        std::vector<NamedJointAccelerationLimit> result;

        for (const auto &item : kDynamics)
        {
            result.push_back(
                {item.name,
                 item.acceleration});
        }

        return result;
    }

    /*
     * 构造完整规划链：
     *
     * CartesianLinePath
     *      ↓
     * IK
     *      ↓
     * velocity timing
     *      ↓
     * cubic trajectory
     */
    Trajectory buildTrajectory(
        const ValidatedUrdfChain<kJointCount> &model,
        const JointPositionConstraints<kJointCount> &constraints)
    {
        const JointVector start_q{
            0.35,
            -0.25,
            0.40,
            -0.80,
            0.45,
            0.25,
            -0.20};

        const JointVector goal_q{
            0.40,
            -0.29,
            0.43,
            -0.75,
            0.41,
            0.28,
            -0.17};

        const Pose start_pose =
            forwardKinematics(
                model,
                start_q);

        const Pose goal_pose =
            forwardKinematics(
                model,
                goal_q);

        CartesianLinePath path{
            start_pose,
            goal_pose};

        CartesianIkPathOptions path_options;

path_options.max_translation_step =
    path.translationLength() / 100.0;

path_options.max_rotation_step =
    path.rotationAngle() / 100.0;

        path_options.max_rotational_joint_step =
            0.1;

        path_options.max_segments =
            100;

        IkOptions ik_options;

        ik_options.position_tolerance =
            1e-6;

        ik_options.rotation_tolerance =
            1e-5;

        const auto cartesian =
            solveCartesianLineIk(
                model,
                constraints,
                path,
                start_q,
                path_options,
                ik_options);

        require(
            cartesian.completed(),
            "Cartesian IK failed");

        JointVelocityLimits<kJointCount>
            velocity_limits{
                model,
                velocityLimits()};

        JointAccelerationLimits<kJointCount>
            acceleration_limits{
                model,
                accelerationLimits()};

        const auto timed =
            parameterizeVelocityLimited(
                model,
                velocity_limits,
                cartesian);

        require(
            timed.completed(),
            "Velocity timing failed");

        return Trajectory{
            model,
            constraints,
            velocity_limits,
            acceleration_limits,
            timed};
    }

    /*
     * 核心测试：
     *
     * 任意时间采样：
     *
     *     t
     *     ↓
     *     q(t)
     *     ↓
     *     FK
     *     ↓
     *     TCP Pose
     *
     * 与：
     *
     * CartesianLinePath.sample(progress)
     *
     * 比较。
     */
    void testCartesianDeviation(
        const ValidatedUrdfChain<kJointCount> &model,
        const JointPositionConstraints<kJointCount> &constraints)
    {
        const Trajectory trajectory =
            buildTrajectory(
                model,
                constraints);

        const JointVector start_q{
            0.35,
            -0.25,
            0.40,
            -0.80,
            0.45,
            0.25,
            -0.20};

        const JointVector goal_q{
            0.40,
            -0.29,
            0.43,
            -0.75,
            0.41,
            0.28,
            -0.17};

        CartesianLinePath cartesian_path{
            forwardKinematics(
                model,
                start_q),

            forwardKinematics(
                model,
                goal_q)};

        const double total_time =
            std::chrono::duration<double>(
                trajectory.duration())
                .count();

        double max_position_error =
            0.0;

        double max_rotation_error =
            0.0;

        constexpr std::size_t samples =
            2000;

        for (std::size_t i = 0;
             i <= samples;
             ++i)
        {
            const double alpha =
                static_cast<double>(i) /
                static_cast<double>(samples);

            const double seconds =
                total_time * alpha;

            const auto state =
                trajectory.sample(
                    std::chrono::duration_cast<Duration>(
                        std::chrono::duration<double>{
                            seconds}));

            const Pose actual =
                forwardKinematics(
                    model,
                    state.position);

            const Pose expected =
                cartesian_path.sample(alpha);

            const double position_error =
                (actual.position -
                 expected.position)
                    .norm();

            const double rotation_error =
                actual.orientation
                    .angularDistance(
                        expected.orientation);

            max_position_error =
                std::max(
                    max_position_error,
                    position_error);

            max_rotation_error =
                std::max(
                    max_rotation_error,
                    rotation_error);
        }

        std::cout
            << std::fixed
            << std::setprecision(12)

            << "[PASS] Cartesian deviation test\n"

            << "  max position deviation [m]: "
            << max_position_error
            << '\n'

            << "  max rotation deviation [rad]: "
            << max_rotation_error
            << '\n';

        /*
         * 第一阶段只测量，不设死工业阈值。
         *
         * 防止因为不同采样密度导致误判。
         */
        require(
            std::isfinite(max_position_error),
            "Position deviation is invalid");

        require(
            std::isfinite(max_rotation_error),
            "Rotation deviation is invalid");
    }

}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr
            << "Usage: "
            << argv[0]
            << " robot.urdf\n";

        return 2;
    }

    try
    {
        auto raw =
            loadUrdfChain(
                argv[1],
                "WAIST_Y_S",
                "L_WRIST_R_S",
                kJointCount);

        ValidatedUrdfChain<kJointCount>
            model{
                raw};

        JointPositionConstraints<kJointCount>
            constraints{
                model};

        testCartesianDeviation(
            model,
            constraints);

        std::cout
            << "All Cartesian deviation tests passed.\n";

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr
            << "Cartesian deviation test failed: "
            << e.what()
            << '\n';

        return 1;
    }
}