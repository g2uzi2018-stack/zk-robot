# TI5 T170C 电机正方向测试记录表

测试日期：　　　　　　　　操作者：　　　　　　　　测试步长：　　　　　　　　

说明：测试程序中 `↑` 表示命令正增量，`↓` 表示命令负增量。请记录现场观察到的实际运动方向，方向结论可填写 `+1` 或 `-1`。参考系为机器人当前的头部的方向为正前方

| 序号 | 业务名称 | 物理关节 | CAN 总线 | Node ID | 起始位置 (rad) | `↑` 实际运动方向 | `↓` 实际运动方向 | 方向结论 | 备注 |
|---:|---|---|---|---:|---:|---|---|---|---|
| 1 | neck_yaw | NECK_Y | head | 30 |  |  |  |  |  |
| 2 | neck_pitch | NECK_P | head | 31 |  |  |  |  |  |
| 3 | neck_roll | NECK_R | head | 32 |  |  |  |  |  |
| 4 | left_shoulder_pitch | L_SHOULDER_P | left_arm | 23 |  | 向后 | 向前 |  |  |
| 5 | left_shoulder_roll | L_SHOULDER_R | left_arm | 24 |  | 向上 | 向下 |  |  |
| 6 | left_shoulder_yaw | L_SHOULDER_Y | left_arm | 25 |  | 向机器人身体内侧旋转 | 向机器人身体外侧旋转 |  |  |
| 7 | left_elbow_yaw | L_ELBOW_Y | left_arm | 26 |  | 向后 | 向前 |  |  |
| 8 | left_wrist_pitch | L_WRIST_P | left_arm | 27 |  | 向机器人身体内侧旋转 | 向机器人身体外侧旋转 |  |  |
| 9 | left_wrist_yaw | L_WRIST_Y | left_arm | 28 |  | 向后                 | 向前                 |  |  |
| 10 | left_wrist_roll | L_WRIST_R | left_arm | 29 |  | 向机器人身体外侧旋转 | 向机器人身体内侧旋转 |  |  |
| 11 | right_shoulder_pitch | R_SHOULDER_P | right_arm | 16 |  | 向前 | 向后 |  |  |
| 12 | right_shoulder_roll | R_SHOULDER_R | right_arm | 17 |  | 向上                 | 向下                 |  |  |
| 13 | right_shoulder_yaw | R_SHOULDER_Y | right_arm | 18 |  | 向机器人身体外侧旋转 | 向机器人身体内侧旋转 |  |  |
| 14 | right_elbow_yaw | R_ELBOW_Y | right_arm | 19 |  | 向前 | 向后 |  |  |
| 15 | right_wrist_pitch | R_WRIST_P | right_arm | 20 |  | 向机器人身体外侧旋转 | 向机器人身体内侧旋转 |  |  |
| 16 | right_wrist_yaw | R_WRIST_Y | right_arm | 21 |  | 向前 | 向后 |  |  |
| 17 | right_wrist_roll | R_WRIST_R | right_arm | 22 |  | 向机器人身体内侧旋转 | 向机器人身体外侧旋转 |  |  |

## 备注

<!-- 可记录异常、限位、反馈延迟或需要复测的关节。 -->

## 启动命令

```shell
cd /home/kuang/workspace/zk_robot
sudo env ZK_ROBOT_CONFIRM_DIRECTION_TEST=YES \
  ./build/tools/ti5_direction_test --delta-rad 0.001
```
