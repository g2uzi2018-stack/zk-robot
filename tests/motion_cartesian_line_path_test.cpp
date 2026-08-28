#include "motion/path/cartesian_line_path.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

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
        using robot::motion::CartesianLinePath;
        using robot::motion::Pose;

        const double pi = std::acos(-1.0);
        const Pose start{
            Eigen::Vector3d{0.1, -0.2, 0.3},
            Eigen::Quaterniond::Identity()};
        const Pose goal{
            Eigen::Vector3d{0.5, 0.2, 0.7},
            Eigen::Quaterniond{
                Eigen::AngleAxisd{0.5 * pi, Eigen::Vector3d::UnitZ()}}};

        const CartesianLinePath path{start, goal};
        const Pose at_start = path.sample(0.0);
        const Pose at_middle = path.sample(0.5);
        const Pose at_goal = path.sample(1.0);

        expect(at_start.position.isApprox(start.position) &&
                   at_start.orientation.isApprox(start.orientation),
               "Cartesian line path start mismatch");
        expect(at_goal.position.isApprox(goal.position) &&
                   at_goal.orientation.isApprox(goal.orientation),
               "Cartesian line path goal mismatch");

        expect(at_middle.position.isApprox(
                   Eigen::Vector3d{0.3, 0.0, 0.5}),
               "Cartesian line path position is not linear");

        const Eigen::Vector3d rotated_x =
            at_middle.orientation * Eigen::Vector3d::UnitX();
        const double half_sqrt_two = std::sqrt(0.5);
        expect(rotated_x.isApprox(
                   Eigen::Vector3d{half_sqrt_two, half_sqrt_two, 0.0}),
               "Cartesian line path orientation is not spherical-linear");

        expect(almostEqual(
                   path.translationLength(),
                   (goal.position - start.position).norm()),
               "Cartesian line path translation length mismatch");
        expect(almostEqual(path.rotationAngle(), 0.5 * pi),
               "Cartesian line path rotation angle mismatch");

        // q 与 -q 表示同一个姿态。路径必须选择最短旋转，不能绕完整一圈。
        Pose equivalent_goal = start;
        equivalent_goal.orientation.coeffs() *= -1.0;
        const CartesianLinePath equivalent_path{start, equivalent_goal};
        expect(almostEqual(equivalent_path.rotationAngle(), 0.0) &&
                   equivalent_path.sample(0.5).orientation.isApprox(
                       start.orientation),
               "Equivalent quaternions produced unnecessary rotation");

        // 合法的非单位四元数会在路径边界归一化。
        Pose scaled_goal = goal;
        scaled_goal.orientation.coeffs() *= 3.0;
        const CartesianLinePath normalized_path{start, scaled_goal};
        expect(almostEqual(normalized_path.goal().orientation.norm(), 1.0),
               "Cartesian path did not normalize its orientation");

        expectThrow<std::out_of_range>(
            [&]() { path.sample(-0.01); },
            "Cartesian path accepted progress below zero");
        expectThrow<std::out_of_range>(
            [&]() { path.sample(1.01); },
            "Cartesian path accepted progress above one");
        expectThrow<std::invalid_argument>(
            [&]() {
                path.sample(std::numeric_limits<double>::quiet_NaN());
            },
            "Cartesian path accepted non-finite progress");

        Pose zero_orientation = goal;
        zero_orientation.orientation.coeffs().setZero();
        expectThrow<std::invalid_argument>(
            [&]() {
                const CartesianLinePath invalid_path{start, zero_orientation};
                static_cast<void>(invalid_path);
            },
            "Cartesian path accepted a zero quaternion");

        Pose non_finite_position = goal;
        non_finite_position.position.x() =
            std::numeric_limits<double>::infinity();
        expectThrow<std::invalid_argument>(
            [&]() {
                const CartesianLinePath invalid_path{
                    start, non_finite_position};
                static_cast<void>(invalid_path);
            },
            "Cartesian path accepted a non-finite position");

        std::cout << "Motion Cartesian line path tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Motion Cartesian line path test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
