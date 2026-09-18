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
 * 测试使用的动态参数。
 *
 * max_velocity：
 *     使用当前真实左臂已有的速度限制。
 *
 * max_acceleration：
 *     当前仓库没有真实 T170C 最大加速度，
 *     所以这里的 40 rad/s^2 只是测试参数。
 *
 * 它只验证：
 *     加速度限制能否真正参与轨迹生成。
 *
 * 不能据此认为真实机器人允许 40 rad/s^2。
 */
struct JointDynamicTestData
{
    const char *name;
    double max_velocity;
    double max_acceleration;
};


constexpr std::array<
    JointDynamicTestData,
    kJointCount>
    kDynamics{{
        {"L_SHOULDER_P", 7.12, 40.0},
        {"L_SHOULDER_R", 7.12, 40.0},
        {"L_SHOULDER_Y", 10.16, 40.0},
        {"L_ELBOW_Y",    10.16, 40.0},
        {"L_WRIST_P",    8.38, 40.0},
        {"L_WRIST_Y",    9.42, 40.0},
        {"L_WRIST_R",    9.42, 40.0},
    }};


void require(
    const bool condition,
    const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(
            message);
    }
}


void requireNear(
    const double actual,
    const double expected,
    const double tolerance,
    const std::string &message)
{
    if (!std::isfinite(actual) ||
        !std::isfinite(expected) ||
        std::abs(actual - expected) >
            tolerance)
    {
        throw std::runtime_error(
            message +
            ": actual=" +
            std::to_string(actual) +
            ", expected=" +
            std::to_string(expected));
    }
}


/*
 * double 秒
 *      ↓
 * JointTrajectory 使用的 Duration
 */
Duration secondsToDuration(
    const double seconds)
{
    return std::chrono::duration_cast<
        Duration>(
        std::chrono::duration<double>{
            seconds});
}


/*
 * 构造速度限制输入。
 */
std::vector<
    NamedJointVelocityLimit>
makeVelocityLimits()
{
    std::vector<
        NamedJointVelocityLimit>
        result;

    result.reserve(
        kJointCount);

    for (const auto &joint :
         kDynamics)
    {
        result.push_back(
            NamedJointVelocityLimit{
                joint.name,
                joint.max_velocity});
    }

    return result;
}


/*
 * 构造加速度限制输入。
 *
 * 注意这里的 max_acceleration
 * 是测试参数。
 */
std::vector<
    NamedJointAccelerationLimit>
makeAccelerationLimits()
{
    std::vector<
        NamedJointAccelerationLimit>
        result;

    result.reserve(
        kJointCount);

    for (const auto &joint :
         kDynamics)
    {
        result.push_back(
            NamedJointAccelerationLimit{
                joint.name,
                joint.max_acceleration});
    }

    return result;
}


/*
 * ============================================================
 * 生成真实左臂 Cartesian IK 路径
 * ============================================================
 *
 * start_q：
 *     真正起点。
 *
 * goal_reference_q：
 *     只用于 FK 生成目标 Pose。
 *
 * 中间：
 *
 *     Pose1
 *     Pose2
 *     ...
 *
 * 对应的关节 q
 * 都由 solveCartesianLineIk() 自己求解。
 */
CartesianIkPathResult<
    kJointCount>
makeCartesianPath(
    const ValidatedUrdfChain<
        kJointCount> &model,
    const JointPositionConstraints<
        kJointCount> &constraints)
{
    const JointVector start_q{
         0.35,
        -0.25,
         0.40,
        -0.80,
         0.45,
         0.25,
        -0.20
    };

    const JointVector goal_reference_q{
         0.40,
        -0.29,
         0.43,
        -0.75,
         0.41,
         0.28,
        -0.17
    };


    const CartesianLinePath path{
        forwardKinematics(
            model,
            start_q),

        forwardKinematics(
            model,
            goal_reference_q)
    };


    CartesianIkPathOptions
        path_options;

    /*
     * 和之前连续 IK 测试一样，
     * 大约生成 9 个 segment。
     */
    path_options.max_translation_step =
        path.translationLength() /
        8.5;

    path_options.max_rotation_step =
        path.rotationAngle() /
        8.5;

    path_options
        .max_rotational_joint_step =
        0.10;

    path_options
        .max_prismatic_joint_step =
        0.01;

    path_options
        .start_position_tolerance =
        1e-8;

    path_options
        .start_rotation_tolerance =
        1e-8;

    path_options.max_segments =
        100;

    path_options.total_time_budget =
        std::chrono::duration<double>{
            5.0};


    IkOptions ik_options;

    ik_options.position_tolerance =
        1e-6;

    ik_options.rotation_tolerance =
        1e-5;

    ik_options.max_iterations =
        400;

    ik_options.time_budget =
        std::chrono::duration<double>{
            0.5};


    const auto result =
        solveCartesianLineIk(
            model,
            constraints,
            path,
            start_q,
            path_options,
            ik_options);


    require(
        result.completed(),
        "Could not build Cartesian "
        "source path: " +
            result.message);

    require(
        result.waypoints.size() > 2,
        "Cartesian source path "
        "contains too few waypoints");

    return result;
}


/*
 * ============================================================
 * 连续 IK 路径
 *      ↓
 * 速度时间参数化
 *      ↓
 * t0 -> q0
 * t1 -> q1
 * ...
 * ============================================================
 */
VelocityTimedJointPath<
    kJointCount>
makeTimedPath(
    const ValidatedUrdfChain<
        kJointCount> &model,
    const JointPositionConstraints<
        kJointCount> &constraints,
    const JointVelocityLimits<
        kJointCount> &velocity_limits)
{
    const auto cartesian =
        makeCartesianPath(
            model,
            constraints);


    VelocityTimeParameterizationOptions
        options;

    options.velocity_scaling =
        1.0;

    /*
     * 设置得很小，
     * 让真实速度限制决定 segment 时间。
     */
    options.minimum_segment_duration =
        std::chrono::duration<double>{
            1e-8};


    const auto result =
        parameterizeVelocityLimited(
            model,
            velocity_limits,
            cartesian,
            options);


    require(
        result.completed(),
        "Velocity timing failed: " +
            result.message);

    return result;
}


/*
 * ============================================================
 * 核心测试：
 *
 * VelocityTimedJointPath
 *        ↓
 * CubicJointTrajectory
 *        ↓
 * sample(t)
 *
 * 验证：
 *
 * 1. 最终 duration 不会比原时间表更短；
 * 2. 所有输入 waypoint 都仍然经过；
 * 3. 起点速度为 0；
 * 4. 终点速度为 0；
 * 5. 任意中间时间都能 sample；
 * 6. 位置不越界；
 * 7. 速度不超 vmax；
 * 8. 加速度不超 amax；
 * 9. waypoint 左右速度连续；
 * 10. waypoint 左右加速度连续。
 * ============================================================
 */
void testRealCubicTrajectory(
    const ValidatedUrdfChain<
        kJointCount> &model,
    const JointPositionConstraints<
        kJointCount> &constraints)
{
    /*
     * q 顺序下的速度限制。
     */
    const JointVelocityLimits<
        kJointCount>
        velocity_limits{
            model,
            makeVelocityLimits()
        };


    /*
     * q 顺序下的加速度限制。
     */
    const JointAccelerationLimits<
        kJointCount>
        acceleration_limits{
            model,
            makeAccelerationLimits()
        };


    /*
     * 上一步生成：
     *
     * t0 -> q0
     * t1 -> q1
     * ...
     */
    const auto timed_path =
        makeTimedPath(
            model,
            constraints,
            velocity_limits);


    CubicJointTrajectoryOptions
        options;

    options.velocity_scaling =
        1.0;

    options.acceleration_scaling =
        1.0;


    /*
     * 真正构造连续轨迹。
     */
    const Trajectory trajectory{
        model,
        constraints,
        velocity_limits,
        acceleration_limits,
        timed_path,
        options
    };


    const double original_duration =
        timed_path.duration.count();


    const double final_duration =
        std::chrono::duration<double>(
            trajectory.duration())
            .count();


    /*
     * 当前 CubicJointTrajectory
     * 只允许把时间拉长，
     * 不允许把上一步时间表主动缩短。
     */
    require(
        final_duration >=
            original_duration -
                1e-12,
        "Cubic trajectory unexpectedly "
        "shortened source timing");


    require(
        trajectory.timeScale() >=
            1.0,
        "Invalid cubic trajectory "
        "time scale");


    /*
     * 构造完成后，
     * 连续轨迹必须已经满足动态限制。
     */
    require(
        trajectory.maxVelocityRatio() <=
            1.0 + 1e-10,
        "Final cubic trajectory "
        "exceeds velocity limit");


    require(
        trajectory.maxAccelerationRatio() <=
            1.0 + 1e-10,
        "Final cubic trajectory "
        "exceeds acceleration limit");


    /*
     * 当前最初的速度时间表非常快，
     * 测试 amax 又故意设置成 40 rad/s²。
     *
     * 正常情况下，
     * 加速度限制应该迫使整条时间轴变长。
     *
     * 这个检查能确认：
     * acceleration limit 真的参与了计算。
     */
    require(
        trajectory.timeScale() >
            1.0 + 1e-6,
        "Expected acceleration limit "
        "to enlarge trajectory time");


    /*
     * ========================================================
     * 起点
     * ========================================================
     */
    const auto start =
        trajectory.sample(
            Duration::zero());


    require(
        !start.finished,
        "Non-zero trajectory unexpectedly "
        "finished at start");


    for (std::size_t joint = 0;
         joint < kJointCount;
         ++joint)
    {
        requireNear(
            start.position[joint],
            timed_path
                .waypoints
                .front()
                .q[joint],
            1e-10,
            "Trajectory start "
            "position mismatch");


        requireNear(
            start.velocity[joint],
            0.0,
            1e-8,
            "Trajectory start "
            "velocity is not zero");
    }


    /*
     * ========================================================
     * 终点
     * ========================================================
     */
    const auto goal =
        trajectory.sample(
            trajectory.duration());


    require(
        goal.finished,
        "Trajectory end is not "
        "marked finished");


    for (std::size_t joint = 0;
         joint < kJointCount;
         ++joint)
    {
        requireNear(
            goal.position[joint],
            timed_path
                .waypoints
                .back()
                .q[joint],
            1e-8,
            "Trajectory goal "
            "position mismatch");


        requireNear(
            goal.velocity[joint],
            0.0,
            1e-6,
            "Trajectory final "
            "velocity is not zero");
    }


    /*
     * ========================================================
     * 检查所有输入 waypoint
     * ========================================================
     *
     * CubicJointTrajectory 使用整体时间缩放：
     *
     *     final_time
     *       =
     *     source_time * timeScale
     *
     * 所以每一个原始 q waypoint
     * 在新的时间点仍必须被精确经过。
     */
    for (std::size_t i = 0;
         i < timed_path.waypoints.size();
         ++i)
    {
        const double waypoint_time =
            timed_path
                .waypoints[i]
                .time_from_start
                .count() *
            trajectory.timeScale();


        const auto point =
            trajectory.sample(
                secondsToDuration(
                    waypoint_time));


        for (std::size_t joint = 0;
             joint < kJointCount;
             ++joint)
        {
            requireNear(
                point.position[joint],
                timed_path
                    .waypoints[i]
                    .q[joint],
                1e-6,
                "Cubic trajectory "
                "missed joint waypoint");
        }
    }


    /*
     * ========================================================
     * 连续时间密集采样
     * ========================================================
     *
     * 从公开 sample(t) 接口
     * 再独立检查一遍。
     */
    constexpr std::size_t
        kSamples = 5000;


    double sampled_max_velocity_ratio =
        0.0;


    double sampled_max_acceleration_ratio =
        0.0;


    for (std::size_t sample_index = 0;
         sample_index <= kSamples;
         ++sample_index)
    {
        const double alpha =
            static_cast<double>(
                sample_index) /
            static_cast<double>(
                kSamples);


        const double seconds =
            final_duration *
            alpha;


        const auto point =
            trajectory.sample(
                secondsToDuration(
                    seconds));


        /*
         * 连续样条内部也不能越过
         * joint position bounds。
         */
        require(
            constraints.contains(
                point.position),
            "Cubic trajectory sample "
            "violates position bounds");


        for (std::size_t joint = 0;
             joint < kJointCount;
             ++joint)
        {
            require(
                std::isfinite(
                    point.position[joint]) &&
                std::isfinite(
                    point.velocity[joint]) &&
                std::isfinite(
                    point.acceleration[joint]),
                "Cubic trajectory returned "
                "non-finite state");


            const double allowed_velocity =
                kDynamics[joint]
                    .max_velocity *
                options.velocity_scaling;


            const double allowed_acceleration =
                kDynamics[joint]
                    .max_acceleration *
                options.acceleration_scaling;


            const double velocity_ratio =
                std::abs(
                    point.velocity[joint]) /
                allowed_velocity;


            const double acceleration_ratio =
                std::abs(
                    point.acceleration[joint]) /
                allowed_acceleration;


            sampled_max_velocity_ratio =
                std::max(
                    sampled_max_velocity_ratio,
                    velocity_ratio);


            sampled_max_acceleration_ratio =
                std::max(
                    sampled_max_acceleration_ratio,
                    acceleration_ratio);


            require(
                velocity_ratio <=
                    1.0 + 1e-8,
                std::string{
                    "Sampled velocity "
                    "exceeded limit: "} +
                    kDynamics[joint].name);


            require(
                acceleration_ratio <=
                    1.0 + 1e-8,
                std::string{
                    "Sampled acceleration "
                    "exceeded limit: "} +
                    kDynamics[joint].name);
        }
    }


    /*
     * ========================================================
     * waypoint 连续性检查
     * ========================================================
     *
     * 在内部 waypoint 的左边、正中、右边
     * 分别 sample。
     *
     * 检查：
     *
     *     velocity 连续
     *     acceleration 连续
     */
    for (std::size_t i = 1;
         i + 1 <
             timed_path.waypoints.size();
         ++i)
    {
        const double waypoint_time =
            timed_path
                .waypoints[i]
                .time_from_start
                .count() *
            trajectory.timeScale();


        const double previous_time =
            timed_path
                .waypoints[i - 1]
                .time_from_start
                .count() *
            trajectory.timeScale();


        const double next_time =
            timed_path
                .waypoints[i + 1]
                .time_from_start
                .count() *
            trajectory.timeScale();


        const double local_interval =
            std::min(
                waypoint_time -
                    previous_time,
                next_time -
                    waypoint_time);


        /*
         * 取非常接近 waypoint 的左右点。
         */
        const double epsilon =
            std::max(
                1e-8,
                local_interval *
                    1e-5);


        const auto left =
            trajectory.sample(
                secondsToDuration(
                    waypoint_time -
                    epsilon));


        const auto exact =
            trajectory.sample(
                secondsToDuration(
                    waypoint_time));


        const auto right =
            trajectory.sample(
                secondsToDuration(
                    waypoint_time +
                    epsilon));


        for (std::size_t joint = 0;
             joint < kJointCount;
             ++joint)
        {
            /*
             * waypoint 本身位置必须正确。
             */
            requireNear(
                exact.position[joint],
                timed_path
                    .waypoints[i]
                    .q[joint],
                1e-6,
                "Internal waypoint "
                "position mismatch");


            /*
             * 左右速度应该足够接近。
             */
            require(
                std::abs(
                    left.velocity[joint] -
                    right.velocity[joint]) <
                    1e-3,
                std::string{
                    "Velocity discontinuity "
                    "near waypoint: "} +
                    kDynamics[joint].name);


            /*
             * 左右加速度也应该足够接近。
             */
            require(
                std::abs(
                    left.acceleration[joint] -
                    right.acceleration[joint]) <
                    1e-2,
                std::string{
                    "Acceleration discontinuity "
                    "near waypoint: "} +
                    kDynamics[joint].name);
        }
    }


    std::cout
        << std::fixed
        << std::setprecision(9)

        << "[PASS] Real left-arm cubic trajectory\n"

        << "  source duration [s]: "
        << original_duration
        << '\n'

        << "  final duration [s]: "
        << final_duration
        << '\n'

        << "  time scale: "
        << trajectory.timeScale()
        << '\n'

        << "  analytic max velocity ratio: "
        << trajectory.maxVelocityRatio()
        << '\n'

        << "  analytic max acceleration ratio: "
        << trajectory.maxAccelerationRatio()
        << '\n'

        << "  sampled max velocity ratio: "
        << sampled_max_velocity_ratio
        << '\n'

        << "  sampled max acceleration ratio: "
        << sampled_max_acceleration_ratio
        << '\n'

        << "  limiting velocity joint: "
        << trajectory.limitingVelocityJoint()
        << '\n'

        << "  limiting acceleration joint: "
        << trajectory.limitingAccelerationJoint()
        << '\n';
}


/*
 * ============================================================
 * 零运动测试
 * ============================================================
 *
 * 只有一个 q：
 *
 *     duration = 0
 *
 * 任意 sample：
 *
 *     position = q
 *     velocity = 0
 *     acceleration = 0
 *     finished = true
 */
void testZeroMotionTrajectory(
    const ValidatedUrdfChain<
        kJointCount> &model,
    const JointPositionConstraints<
        kJointCount> &constraints)
{
    const JointVector q{
         0.35,
        -0.25,
         0.40,
        -0.80,
         0.45,
         0.25,
        -0.20
    };


    const Pose pose =
        forwardKinematics(
            model,
            q);


    const CartesianLinePath path{
        pose,
        pose
    };


    const auto cartesian =
        solveCartesianLineIk(
            model,
            constraints,
            path,
            q);


    require(
        cartesian.completed(),
        "Could not build "
        "zero Cartesian path");


    const JointVelocityLimits<
        kJointCount>
        velocity_limits{
            model,
            makeVelocityLimits()
        };


    const JointAccelerationLimits<
        kJointCount>
        acceleration_limits{
            model,
            makeAccelerationLimits()
        };


    const auto timed =
        parameterizeVelocityLimited(
            model,
            velocity_limits,
            cartesian);


    require(
        timed.completed(),
        "Could not time zero path");


    const Trajectory trajectory{
        model,
        constraints,
        velocity_limits,
        acceleration_limits,
        timed
    };


    require(
        trajectory.duration() ==
            Duration::zero(),
        "Zero trajectory has "
        "non-zero duration");


    const auto point =
        trajectory.sample(
            std::chrono::seconds{
                1});


    require(
        point.finished,
        "Zero trajectory should "
        "always be finished");


    for (std::size_t joint = 0;
         joint < kJointCount;
         ++joint)
    {
        requireNear(
            point.position[joint],
            q[joint],
            0.0,
            "Zero trajectory "
            "position changed");


        requireNear(
            point.velocity[joint],
            0.0,
            0.0,
            "Zero trajectory "
            "has velocity");


        requireNear(
            point.acceleration[joint],
            0.0,
            0.0,
            "Zero trajectory "
            "has acceleration");
    }


    std::cout
        << "[PASS] Zero-motion "
        "cubic trajectory\n";
}

} // namespace


int main(
    int argc,
    char *argv[])
{
    if (argc != 2)
    {
        std::cerr
            << "Usage: "
            << "motion_left_arm_cubic_trajectory_test "
            << "<robot.urdf>\n";

        return 2;
    }


    try
    {
        const UrdfChain raw_chain =
            loadUrdfChain(
                argv[1],
                "WAIST_Y_S",
                "L_WRIST_R_S",
                kJointCount);


        const ValidatedUrdfChain<
            kJointCount>
            model{
                raw_chain};


        const JointPositionConstraints<
            kJointCount>
            constraints{
                model};


        testRealCubicTrajectory(
            model,
            constraints);


        testZeroMotionTrajectory(
            model,
            constraints);


        std::cout
            << "\nAll cubic joint "
            << "trajectory tests passed.\n";

        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "Cubic joint trajectory "
            << "test failed: "
            << error.what()
            << '\n';

        return 1;
    }
    catch (...)
    {
        std::cerr
            << "Cubic joint trajectory "
            << "test failed: "
            << "unknown exception\n";

        return 1;
    }
}