#include "motion/kinematics/urdf_chain.hpp"
#include "motion/planning/cartesian_ik_path.hpp"
#include "motion/planning/velocity_time_parameterization.hpp"

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
using JointVector = std::array<double, kJointCount>;

struct ExpectedVelocity
{
    const char *name;
    double max_velocity;
};

/*
 * 当前真实左臂 URDF 中声明的关节最大速度。
 *
 * Revolute 关节单位均为 rad/s。
 *
 * 这里显式写出来是为了做回归测试；
 * 生产代码以后应从机器人配置/模型初始化阶段提供，
 * 而不是把这些值硬编码在时间参数化算法内部。
 */
constexpr std::array<ExpectedVelocity, kJointCount>
    kVelocityLimits{{
        {"L_SHOULDER_P", 7.12},
        {"L_SHOULDER_R", 7.12},
        {"L_SHOULDER_Y", 10.16},
        {"L_ELBOW_Y",    10.16},
        {"L_WRIST_P",    8.38},
        {"L_WRIST_Y",    9.42},
        {"L_WRIST_R",    9.42},
    }};

void require(
    const bool condition,
    const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
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
        std::abs(actual - expected) > tolerance)
    {
        throw std::runtime_error(
            message +
            ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected));
    }
}

/*
 * 把测试中的“名字 + 最大速度”
 * 转成上一轮接口要求的 vector。
 */
std::vector<NamedJointVelocityLimit>
makeVelocityConfiguration()
{
    std::vector<NamedJointVelocityLimit> result;

    result.reserve(kVelocityLimits.size());

    for (const auto &limit : kVelocityLimits)
    {
        result.push_back(
            NamedJointVelocityLimit{
                limit.name,
                limit.max_velocity});
    }

    return result;
}

/*
 * 构造与上一轮相同的真实左臂 Cartesian 路径。
 *
 * start_q：
 *     真正的起点。
 *
 * reference_goal_q：
 *     只用 FK 生成目标 Pose。
 *
 * 中间所有关节 waypoint
 * 仍然由 solveCartesianLineIk() 自己求出来。
 */
CartesianIkPathResult<kJointCount>
makeRealCartesianPath(
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
        -0.20
    };

    const JointVector reference_goal_q{
         0.40,
        -0.29,
         0.43,
        -0.75,
         0.41,
         0.28,
        -0.17
    };

    const CartesianLinePath path{
        forwardKinematics(model, start_q),
        forwardKinematics(model, reference_goal_q)};

    CartesianIkPathOptions path_options;

    /*
     * 与上一轮测试一样，
     * 故意得到大约 9 个 segment。
     */
    path_options.max_translation_step =
        path.translationLength() / 8.5;

    path_options.max_rotation_step =
        path.rotationAngle() / 8.5;

    path_options.max_rotational_joint_step =
        0.10;

    path_options.max_prismatic_joint_step =
        0.01;

    path_options.start_position_tolerance =
        1e-8;

    path_options.start_rotation_tolerance =
        1e-8;

    path_options.max_segments =
        100;

    path_options.total_time_budget =
        std::chrono::duration<double>{5.0};

    IkOptions ik_options;

    ik_options.position_tolerance =
        1e-6;

    ik_options.rotation_tolerance =
        1e-5;

    ik_options.max_iterations =
        400;

    ik_options.time_budget =
        std::chrono::duration<double>{0.5};

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
        "Could not prepare real Cartesian IK path: " +
            result.message);

    require(
        result.waypoints.size() > 2,
        "Real Cartesian path contains too few waypoints");

    return result;
}

/*
 * 独立计算某一段在速度限制下至少需要多少时间。
 *
 * 这是测试侧重新计算，
 * 用来验证 parameterizeVelocityLimited() 的输出。
 *
 * 对每个关节：
 *
 *     abs(delta_q) / allowed_velocity
 *
 * 找最大的那个。
 */
double expectedSegmentSeconds(
    const JointVector &from,
    const JointVector &to,
    const double velocity_scaling,
    const double minimum_segment_seconds)
{
    double required =
        0.0;

    for (std::size_t i = 0;
         i < kJointCount;
         ++i)
    {
        const double delta =
            std::abs(
                to[i] - from[i]);

        const double allowed_velocity =
            kVelocityLimits[i].max_velocity *
            velocity_scaling;

        required =
            std::max(
                required,
                delta / allowed_velocity);
    }

    return std::max(
        required,
        minimum_segment_seconds);
}

/*
 * 核心测试：
 *
 * Cartesian IK：
 *
 *     q0 q1 q2 ... qN
 *
 *       ↓ velocity time parameterization
 *
 *     t0→q0
 *     t1→q1
 *     t2→q2
 *     ...
 *
 * 逐段检查：
 *
 *     1. q 没有被修改；
 *     2. progress 没有被修改；
 *     3. 时间严格递增；
 *     4. segment 时间等于理论最短时间；
 *     5. 每个关节实际速度 <= vmax；
 *     6. 整条 duration 等于最后 waypoint 时间。
 */
void testRealVelocityTiming(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
{
    const auto source_path =
        makeRealCartesianPath(
            model,
            constraints);

    const JointVelocityLimits<kJointCount>
        velocity_limits{
            model,
            makeVelocityConfiguration()};

    VelocityTimeParameterizationOptions options;

    options.velocity_scaling =
        1.0;

    /*
     * 设置得足够小，
     * 保证本测试主要由真实速度限制决定，
     * 而不是 minimum_segment_duration 决定。
     */
    options.minimum_segment_duration =
        std::chrono::duration<double>{1e-8};

    const auto timed =
        parameterizeVelocityLimited(
            model,
            velocity_limits,
            source_path,
            options);

    require(
        timed.completed(),
        "Velocity parameterization failed: " +
            timed.message);

    require(
        timed.waypoints.size() ==
            source_path.waypoints.size(),
        "Velocity timing changed waypoint count");

    require(
        !timed.waypoints.empty(),
        "Velocity timing returned empty path");

    requireNear(
        timed.waypoints.front()
            .time_from_start.count(),
        0.0,
        0.0,
        "First waypoint time must be zero");

    double expected_total =
        0.0;

    double independently_measured_max_ratio =
        0.0;

    std::string independent_limiting_joint;

    for (std::size_t i = 0;
         i < timed.waypoints.size();
         ++i)
    {
        /*
         * 时间参数化不能改变几何/IK 路径。
         */
        require(
            timed.waypoints[i].q ==
                source_path.waypoints[i].q,
            "Velocity timing changed joint waypoint");

        requireNear(
            timed.waypoints[i].progress,
            source_path.waypoints[i].progress,
            0.0,
            "Velocity timing changed Cartesian progress");

        if (i == 0)
        {
            continue;
        }

        const auto &previous =
            timed.waypoints[i - 1];

        const auto &current =
            timed.waypoints[i];

        const double actual_segment_seconds =
            current.time_from_start.count() -
            previous.time_from_start.count();

        require(
            actual_segment_seconds > 0.0,
            "Timed waypoints are not strictly increasing");

        const double expected_segment_seconds =
            expectedSegmentSeconds(
                previous.q,
                current.q,
                options.velocity_scaling,
                options.minimum_segment_duration.count());

        requireNear(
            actual_segment_seconds,
            expected_segment_seconds,
            1e-12,
            "Unexpected velocity-limited segment time");

        expected_total +=
            expected_segment_seconds;

        /*
         * 再从最终时间反算实际关节速度。
         */
        for (std::size_t joint = 0;
             joint < kJointCount;
             ++joint)
        {
            const double delta =
                std::abs(
                    current.q[joint] -
                    previous.q[joint]);

            const double actual_velocity =
                delta /
                actual_segment_seconds;

            const double allowed_velocity =
                kVelocityLimits[joint]
                    .max_velocity *
                options.velocity_scaling;

            require(
                actual_velocity <=
                    allowed_velocity *
                        (1.0 + 1e-12),
                std::string{
                    "Joint velocity exceeded limit: "} +
                    kVelocityLimits[joint].name);

            const double ratio =
                actual_velocity /
                allowed_velocity;

            if (ratio >
                independently_measured_max_ratio)
            {
                independently_measured_max_ratio =
                    ratio;

                independent_limiting_joint =
                    kVelocityLimits[joint].name;
            }
        }
    }

    requireNear(
        timed.duration.count(),
        expected_total,
        1e-12,
        "Total timed-path duration mismatch");

    requireNear(
        timed.waypoints.back()
            .time_from_start.count(),
        timed.duration.count(),
        1e-12,
        "Last waypoint time does not equal duration");

    requireNear(
        timed.max_velocity_ratio,
        independently_measured_max_ratio,
        1e-12,
        "Reported maximum velocity ratio mismatch");

    require(
        timed.limiting_joint_name ==
            independent_limiting_joint,
        "Reported limiting joint mismatch");

    /*
     * 因为每个 segment 都取 max(delta/vmax)，
     * 并且 minimum duration 没有成为瓶颈，
     * 至少应该有某个关节正好接近允许速度。
     */
    require(
        timed.max_velocity_ratio >
            0.999999 &&
        timed.max_velocity_ratio <=
            1.0 + 1e-12,
        "Expected one joint to determine minimum timing");

    std::cout
        << std::fixed
        << std::setprecision(9)
        << "[PASS] Real left-arm velocity timing\n"
        << "  waypoints: "
        << timed.waypoints.size()
        << '\n'
        << "  duration [s]: "
        << timed.duration.count()
        << '\n'
        << "  max velocity ratio: "
        << timed.max_velocity_ratio
        << '\n'
        << "  limiting joint: "
        << timed.limiting_joint_name
        << '\n';
}

/*
 * 验证 velocity_scaling 的机制。
 *
 * 速度倍率从：
 *
 *     100%
 *
 * 降到：
 *
 *     50%
 *
 * 对同样一条 q 路径，
 * 在 minimum duration 不起作用时，
 * 每一段理论时间应变成原来的 2 倍。
 */
void testVelocityScaling(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
{
    const auto source_path =
        makeRealCartesianPath(
            model,
            constraints);

    const JointVelocityLimits<kJointCount>
        limits{
            model,
            makeVelocityConfiguration()};

    VelocityTimeParameterizationOptions full_options;

    full_options.velocity_scaling =
        1.0;

    full_options.minimum_segment_duration =
        std::chrono::duration<double>{1e-8};

    auto half_options =
        full_options;

    half_options.velocity_scaling =
        0.5;

    const auto full =
        parameterizeVelocityLimited(
            model,
            limits,
            source_path,
            full_options);

    const auto half =
        parameterizeVelocityLimited(
            model,
            limits,
            source_path,
            half_options);

    require(
        full.completed() &&
            half.completed(),
        "Velocity-scaling parameterization failed");

    requireNear(
        half.duration.count(),
        full.duration.count() * 2.0,
        1e-12,
        "50% velocity scaling should double path duration");

    /*
     * 时间变了，但是路径本身绝对不能变。
     */
    require(
        half.waypoints.size() ==
            full.waypoints.size(),
        "Velocity scaling changed waypoint count");

    for (std::size_t i = 0;
         i < full.waypoints.size();
         ++i)
    {
        require(
            full.waypoints[i].q ==
                half.waypoints[i].q,
            "Velocity scaling changed joint path");

        requireNear(
            full.waypoints[i].progress,
            half.waypoints[i].progress,
            0.0,
            "Velocity scaling changed Cartesian path");
    }

    std::cout
        << std::fixed
        << std::setprecision(9)
        << "[PASS] Velocity scaling\n"
        << "  100% duration [s]: "
        << full.duration.count()
        << '\n'
        << "  50% duration [s]: "
        << half.duration.count()
        << '\n';
}

/*
 * 零运动路径应该仍然是：
 *
 *     time = 0
 *     q = start_q
 *
 * 总持续时间为 0。
 */
void testZeroMotionTiming(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
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
        pose};

    const auto source =
        solveCartesianLineIk(
            model,
            constraints,
            path,
            q);

    require(
        source.completed(),
        "Could not create zero-motion Cartesian path");

    const JointVelocityLimits<kJointCount>
        limits{
            model,
            makeVelocityConfiguration()};

    const auto result =
        parameterizeVelocityLimited(
            model,
            limits,
            source);

    require(
        result.completed(),
        "Zero-motion timing failed");

    require(
        result.waypoints.size() == 1,
        "Zero-motion timing changed waypoint count");

    requireNear(
        result.duration.count(),
        0.0,
        0.0,
        "Zero-motion duration must be zero");

    requireNear(
        result.waypoints.front()
            .time_from_start.count(),
        0.0,
        0.0,
        "Zero-motion waypoint time must be zero");

    std::cout
        << "[PASS] Zero-motion velocity timing\n";
}

/*
 * 一个没有完整求完的 Cartesian path
 * 不能被偷偷拿去生成时间表。
 */
void testIncompletePathRejected(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
{
    auto source =
        makeRealCartesianPath(
            model,
            constraints);

    source.status =
        CartesianIkPathStatus::JointJump;

    const JointVelocityLimits<kJointCount>
        limits{
            model,
            makeVelocityConfiguration()};

    const auto result =
        parameterizeVelocityLimited(
            model,
            limits,
            source);

    require(
        result.status ==
            VelocityTimeParameterizationStatus::
                SourcePathIncomplete,
        "Incomplete Cartesian path was not rejected");

    std::cout
        << "[PASS] Incomplete source path rejected\n";
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr
            << "Usage: motion_left_arm_velocity_time_test <robot.urdf>\n";

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

        const ValidatedUrdfChain<kJointCount>
            model{
                raw_chain};

        const JointPositionConstraints<kJointCount>
            constraints{
                model};

        testRealVelocityTiming(
            model,
            constraints);

        testVelocityScaling(
            model,
            constraints);

        testZeroMotionTiming(
            model,
            constraints);

        testIncompletePathRejected(
            model,
            constraints);

        std::cout
            << "\nAll velocity time-parameterization tests passed.\n";

        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "Velocity time-parameterization test failed: "
            << error.what()
            << '\n';

        return 1;
    }
    catch (...)
    {
        std::cerr
            << "Velocity time-parameterization test failed: "
            << "unknown exception\n";

        return 1;
    }
}