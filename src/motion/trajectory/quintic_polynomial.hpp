#pragma once

#include <array>

namespace robot::motion
{

    using QuinticCoefficients = std::array<double, 6>;

    // 在归一化时间 u = t / T, u ∈ [0, 1] 下，
    // 根据起点/终点的位置、速度、加速度生成五次多项式系数：
    //
    // q(u) = a0 + a1*u + a2*u² + a3*u³ + a4*u⁴ + a5*u⁵
    /**
     * @param start_position 起点位置
     * @param start_velocity 起点速度
     * @param start_acceleration 起点加速度
     * @param goal_position 终点位置
     * @param goal_velocity 终点速度
     * @param goal_acceleration 终点加速度
     * @param duration_seconds 轨迹总时间
     * @return 五次多项式系数
     */
    inline QuinticCoefficients makeNormalizedQuinticCoefficients(
        const double start_position,
        const double start_velocity,
        const double start_acceleration,
        const double goal_position,
        const double goal_velocity,
        const double goal_acceleration,
        const double duration_seconds)
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
        const double T = duration_seconds;
        const double T2 = T * T;

        const double displacement =
            goal_position - start_position;

        const double a0 = start_position;

        const double a1 = start_velocity * T;

        const double a2 = 0.5 * start_acceleration * T2;

        const double a3 = 10.0 * displacement - (6.0 * start_velocity + 4.0 * goal_velocity) * T - (1.5 * start_acceleration - 0.5 * goal_acceleration) * T2;

        const double a4 = -15.0 * displacement + (8.0 * start_velocity + 7.0 * goal_velocity) * T + (1.5 * start_acceleration - goal_acceleration) * T2;

        const double a5 = 6.0 * displacement - 3.0 * (start_velocity + goal_velocity) * T - 0.5 * (start_acceleration - goal_acceleration) * T2;

        return {a0, a1, a2, a3, a4, a5};
    }

} // namespace robot::motion