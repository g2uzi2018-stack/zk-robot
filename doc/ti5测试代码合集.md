# TI5 测试工具合集

本文只记录当前 `tools/ti5/` 目录下保留的两个 TI5 工具。它们属于实机调试程序，
不会由 `ctest` 自动运行；软件流程通过不等同于机器人实机验收通过。

记录日期：2026-09-10。工程路径：`/home/kuang/workspace/zk_robot`。

## 工具总览

| 工具 | 代码入口 | 主要用途 |
| --- | --- | --- |
| `ti5_joint_cli` | [`tools/ti5/ti5_joint_cli.cpp`](../tools/ti5/ti5_joint_cli.cpp) | 交互式查看和小步调整 TI5 头部、双臂、双手及腰部关节 |
| `ti5_zero_home` | [`tools/ti5/ti5_zero_home.cpp`](../tools/ti5/ti5_zero_home.cpp) | 头部和双臂 17 轴回零、缓降 STOP、记录及运行特定点 |

## `ti5_joint_cli`

### 用途和范围

这是一个交互式 TI5 关节控制台。启动后通过 Node ID 自动发现四条本体逻辑总线：

- `waist_fold`
- `head`
- `left_arm`
- `right_arm`

主菜单包含六个部件：

1. 头部：`neck_yaw`、`neck_pitch`、`neck_roll`
2. 左臂：7 个关节
3. 右臂：7 个关节
4. 左手：6 个 raw 通道
5. 右手：6 个 raw 通道
6. 腰部：`waist_yaw`、`fold_p3`、`fold_p2`、`fold_p1`、`fold_r`

头部、双臂和腰部按弧度调整，灵巧手按 raw 位置单位调整。控制循环会周期性读取反馈
并刷新正在运行的目标；按 `q` 逐级返回或退出。

### 编译和配置检查

```bash
cd /home/kuang/workspace/zk_robot
cmake --build build --target ti5_joint_cli -j2
./build/tools/ti5_joint_cli --dry-run
```

`--dry-run` 只加载并检查 TI5 配置，不打开 CAN。

### 实机操作

```bash
./build/tools/ti5_joint_cli
```

主菜单使用 `1`～`6` 选择部件；进入部件后用数字键选择关节，方向键或 `+`/`-` 调整目标，
`q` 返回上一级。头部、双臂和腰部每步调整 `0.05 rad`，灵巧手步长通过参数设置：

```bash
./build/tools/ti5_joint_cli \
  --hand-step-raw 50 \
  --initial-raw 30000 \
  --speed-raw 5
```

当灵巧手配置没有开放控制时，还需要使用 `--commission`，并设置：

```bash
export ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST=YES
```

退出时头部请求并确认 `mode=0`；双臂和腰部保留最后的位置目标。`mode=0` 只代表停止运行
模式，不代表去使能、抱闸或释放负载，因此操作时必须确保机械支撑和急停可用。

## `ti5_zero_home`

### 用途和范围

该工具只控制头部 3 轴和双臂 14 轴，共 17 个本体关节；固定排除：

- `waist_yaw`
- `Fold_P1/P2/P3/R`
- 左右灵巧手

程序提供四个菜单动作：

| 菜单 | 行为 |
| --- | --- |
| `1` | 头部和双臂回到 CAN 电机角 `0 rad` 并保持 |
| `2` | 双臂经两个中间点缓慢下放；操作者托稳后发送 `0x02 STOP` |
| `3` | 只读记录当前 17 轴姿态到 `config/ti5/t170c/recorded_waypoint.txt` |
| `4` | 按已记录特定点执行五次曲线运动并保持 |

特定点文件保存的是 CAN 输出端电机角，不经过 `kinematics.yaml` 的模型坐标换算。
程序会检查软件限位、驱动器限位、反馈新鲜度和目标路径；腰部和折叠机构不在本工具范围内。

### 编译和无动作检查

```bash
cd /home/kuang/workspace/zk_robot
cmake --build build --target ti5_zero_home -j8
./build/tools/ti5_zero_home --dry-run
```

`--dry-run` 只打印测试计划，不打开 CAN、不发送控制帧。`--no-bring-up` 可用于要求本体
CAN 已经处于 UP 且为 1 Mbps 的场景。

### 实机操作

```bash
cd ~/workspace/zk_robot
sudo env ZK_ROBOT_CONFIRM_TI5_TEST=YES ./build/tools/ti5_zero_home
```

执行菜单 `2` 时，双臂会在第二中间点保持，必须先可靠托住双臂，再确认最终 STOP。
`STOP` 只表示请求驱动器进入 `mode=0`，不代表去使能、抱闸或一定能承受重力；
菜单 `1` 和 `4` 完成后会保留位置保持，不能把退出程序当作自动安全停机。

## 通用操作前提

- 阅读本文件和对应源码中的提示，确认当前操作对象和动作范围。
- 关闭其他 TI5/CAN 控制程序，保证测试进程独占本体 CAN。
- 实机动作前确认急停可达，双臂测试时安排人员或机械结构可靠承托。
- 不要把 `recorded_waypoint.txt` 等机器人主机运行时数据提交到版本库。
