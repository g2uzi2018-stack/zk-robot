#pragma once

#include <array>
#include <chrono>
#include <cstddef>

namespace robot::motion
{

// 一条关节轨迹在某个时刻的采样结果。
//
// position:
//   当前时刻各关节应该达到的真实位置。
//
// velocity:
//   当前时刻各关节的轨迹目标速度。
//
// acceleration:
//   当前时刻各关节的轨迹目标加速度。
//
// finished:
//   当前采样时间是否已经达到或超过轨迹终点。
template <std::size_t N>
struct JointTrajectoryPoint
{
    std::array<double, N> position{};
    std::array<double, N> velocity{};
    std::array<double, N> acceleration{};

    bool finished{false};
};

// 可执行关节轨迹接口。
//
// JointTrajectory 只描述：
//   给定轨迹执行时间 elapsed，
//   当前各关节应该处于什么状态。
//
// 它不负责：
//   - 控制线程
//   - CAN
//   - Controller
//   - 正运动学 / 逆运动学
//   - 笛卡尔空间规划
//   - 路径规划
//
// Executor 只依赖这个接口，
// 不关心具体轨迹是五次多项式、样条还是其他形式。
template <std::size_t N>
class JointTrajectory
{
public:
    using Clock = std::chrono::steady_clock;
    using Duration = Clock::duration;
    using Point = JointTrajectoryPoint<N>;

    virtual ~JointTrajectory() = default;

    // 对已经生成好的轨迹进行时间采样。
    //
    // elapsed:这条轨迹已经执行多久了
    //   从当前轨迹开始执行到现在经过的时间。
    //
    // 该函数必须快速、有界，不进行阻塞操作。
    virtual Point sample(Duration elapsed) const = 0;

    // 返回整条轨迹的持续时间。
    virtual Duration duration() const noexcept = 0;
};
} // namespace robot::motion
