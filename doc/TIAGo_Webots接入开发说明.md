# TIAGo Webots 接入开发说明

## 1. 当前状态与边界

本文客观记录本机 TIAGo++ Webots/SocketCAN 仿真的资源、接口、启动方式和验收口径。

```text
/home/kuang/workspace/tiago_can_webots   Webots 模型、CAN 网关、映射和验证工具
```

截至 2026-08-14：

- `tiago_can_webots` 已映射全部 `23` 个主动关节，使用 `vcan0..vcan10`；
- CAN v1 仍是 Classical CAN、11 位标准 ID、固定 8 字节帧，线协议未改变；
- base、torso、head 分别使用 `vcan8`、`vcan9`、`vcan10`；
- Webots Robot 根节点为 `locked FALSE`，两轮收到 vcan8 velocity 命令后可产生
  真实底盘位移；

## 2. 权威数据

| 数据 | 权威文件 |
|---|---|
| Webots 场景和地面接触参数 | `tiago_can_webots/worlds/tiago_dual_can.wbt` |
| Motor、PositionSensor、物理根和初始姿态 | `tiago_can_webots/protos/TiagoDual.proto` |
| 关节类型、机械限位和机械链 | `tiago_can_webots/generated/tiago_dual.urdf` |
| vCAN、Node ID、控制模式和编码器标定 | `tiago_can_webots/config/joint_mapping.yaml` |
| CAN v1 定义 | `tiago_can_webots/config/can_protocol.yaml` |

`joint_mapping.yaml` 是仿真映射真源；Node ID、vCAN、控制模式和单位均以它为准。

## 3. 构建和启动

构建并验证仿真后端：

```bash
cd /home/kuang/workspace/tiago_can_webots
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
.venv/bin/pytest -q
.venv/bin/python scripts/validate_webots_model.py
```

终端 1 创建 11 条 vCAN 并启动正式场景：

```bash
cd /home/kuang/workspace/tiago_can_webots
./scripts/setup_vcan.sh          # 创建并启用 vcan0..vcan10
./scripts/launch_webots.sh
```

终端 2 使用正式入口连接已经运行的 `TIAGoDual`：

```bash
cd /home/kuang/workspace/tiago_can_webots
./scripts/run_gateway.sh
```

`run_gateway.sh` 会调用 `/usr/local/webots/webots-controller`，启动一个
`build/can_motor_gateway`，读取 `config/joint_mapping.yaml` 并同时打开
`vcan0..vcan10`。它不会创建 vCAN、启动 Webots 或主动发送运动目标。

网关启动时所有节点在 CAN 逻辑上都是 disabled；disabled 状态拒绝运动命令。
Webots 物理电机仍保持导入姿态，以防重力使上身塌落。这种“逻辑禁用、仿真保持”
是当前后端的安全约定。

## 4. 23 个节点映射

| vCAN | 关节 | Node | 命令 ID | 反馈 ID | 类型 / 正常模式 | 单位 |
|---|---|---:|---:|---:|---|---|
| `vcan0` | `arm_left_1_joint` | `0x01` | `0x101` | `0x181` | revolute / position | rad |
| `vcan0` | `arm_left_2_joint` | `0x02` | `0x102` | `0x182` | revolute / position | rad |
| `vcan1` | `arm_left_3_joint` | `0x03` | `0x103` | `0x183` | revolute / position | rad |
| `vcan1` | `arm_left_4_joint` | `0x04` | `0x104` | `0x184` | revolute / position | rad |
| `vcan2` | `arm_left_5_joint` | `0x05` | `0x105` | `0x185` | revolute / position | rad |
| `vcan2` | `arm_left_6_joint` | `0x06` | `0x106` | `0x186` | revolute / position | rad |
| `vcan2` | `arm_left_7_joint` | `0x07` | `0x107` | `0x187` | revolute / position | rad |
| `vcan6` | `gripper_left_right_finger_joint` | `0x08` | `0x108` | `0x188` | prismatic / position | m |
| `vcan6` | `gripper_left_left_finger_joint` | `0x09` | `0x109` | `0x189` | prismatic / position | m |
| `vcan8` | `wheel_right_joint` | `0x0A` | `0x10A` | `0x18A` | continuous / velocity | rad/s |
| `vcan8` | `wheel_left_joint` | `0x0B` | `0x10B` | `0x18B` | continuous / velocity | rad/s |
| `vcan9` | `torso_lift_joint` | `0x0C` | `0x10C` | `0x18C` | prismatic / position | m |
| `vcan10` | `head_1_joint` | `0x0D` | `0x10D` | `0x18D` | revolute / position | rad |
| `vcan10` | `head_2_joint` | `0x0E` | `0x10E` | `0x18E` | revolute / position | rad |
| `vcan3` | `arm_right_1_joint` | `0x11` | `0x111` | `0x191` | revolute / position | rad |
| `vcan3` | `arm_right_2_joint` | `0x12` | `0x112` | `0x192` | revolute / position | rad |
| `vcan4` | `arm_right_3_joint` | `0x13` | `0x113` | `0x193` | revolute / position | rad |
| `vcan4` | `arm_right_4_joint` | `0x14` | `0x114` | `0x194` | revolute / position | rad |
| `vcan5` | `arm_right_5_joint` | `0x15` | `0x115` | `0x195` | revolute / position | rad |
| `vcan5` | `arm_right_6_joint` | `0x16` | `0x116` | `0x196` | revolute / position | rad |
| `vcan5` | `arm_right_7_joint` | `0x17` | `0x117` | `0x197` | revolute / position | rad |
| `vcan7` | `gripper_right_right_finger_joint` | `0x18` | `0x118` | `0x198` | prismatic / position | m |
| `vcan7` | `gripper_right_left_finger_joint` | `0x19` | `0x119` | `0x199` | prismatic / position | m |

每个主动关节只出现一次，Node ID 全局唯一。把合法 Node ID 发到错误 vCAN 时，
网关必须忽略该帧。

## 5. CAN v1 线协议

```text
command_id  = 0x100 + node_id
feedback_id = 0x180 + node_id
status_id   = 0x200 + node_id  # 保留，不单独发送
```

命令帧：

```text
byte 0..3  int32   position counts，或 velocity counts/s
byte 4..5  uint16  最大速度 counts/s
byte 6     uint8   命令码
byte 7     uint8   flags，当前必须为 0
```

| 命令码 | 含义 |
|---:|---|
| `0x01` | enable |
| `0x02` | disable |
| `0x03` | clear fault，清除后仍为 disabled |
| `0x04` | set zero，仅修改本次运行零点 |
| `0x10` | position |
| `0x11` | velocity |
| `0x12` | stop |
| `0x20` | query status |
| `0x21` | query position |
| `0x22` | query velocity |

反馈帧：

```text
byte 0..3  int32  position counts
byte 4..5  int16  velocity counts/s，超出线宽时饱和
byte 6 bit0       enabled
byte 6 bit1       faulted
byte 6 bit2       timed_out
byte 7            fault code：0=无故障，1=限位故障
```

网关通常每 `20 ms` 发布反馈，query 会立即触发反馈。推荐每 `100 ms` 刷新运动
命令；超过 `500 ms` 未刷新时进入 Hold 并置 `timed_out=1`。query 不刷新看门狗。

## 6. 单位和关节语义

- Arm/head：`revolute + position + radian`，仿真默认
  `4096 counts/rev`、`gear_ratio=1`、`direction=+1`、`zero_offset=0`。
- Gripper/torso：`prismatic + position + meter`，mapping 明确使用
  `motion: linear`、`unit: meter`、`counts_per_meter`；不要用 `*_rad` 或
  `counts_per_motor_revolution` 隐式表示直线位移。
- Wheels：`continuous + velocity + radian`。正常控制使用 `0x11`；没有有限
  position min/max，也不应给轮子发送普通 position 命令。

`torso_lift_joint` 是竖直直线升降关节，不是腰部 yaw。以上编码器值都标记为
`simulation_default`，不能直接用于 PAL 实机。

## 7. 底盘真实位移

生成脚本会把上游 URDF 的空 `base_footprint` 包装移除，把有质量、惯性和碰撞体的
`base_link` 提升为 Webots Robot 物理根，同时保留原来的 `0.0985 m` 离地高度。
PROTO 中两只驱动轮使用 `tiago_drive_wheel` 接触材料，四个固定球形脚轮使用低摩擦
`tiago_caster`；正式 world 必须提供与 `tiago_floor` 的 ContactProperties。

端到端验证命令：

```bash
cd /home/kuang/workspace/tiago_can_webots
bash tests/test_base_displacement.sh
```

该测试启动独立 Webots 场景和正式 `can_motor_gateway`，经 vcan8 对两轮执行
clear fault、enable、同向 `+0.6 rad/s`、stop、disable，并由 Supervisor 测量
Robot 世界坐标。2026-08-14 本机实测 2.5 秒平移约 `0.1478 m`，停止后轮速反馈为 0。

这证明了 vcan8 到物理位移的直行链路；仓库不包含导航、里程计、复杂地面或高速
动力学验收。网关对左右轮分别执行 stop、disable 和 `500 ms` 命令看门狗。

## 8. 仓库资源分布

| 路径 | 内容 |
|---|---|
| `worlds/` | 正式 Webots world；地面和移动底盘 ContactProperties 在这里 |
| `protos/` | 由 PAL URDF 转换并补丁化的 `TiagoDual.proto` |
| `generated/` | 固定 PAL 模型源展开得到的 TIAGo++ URDF |
| `third_party/pal/` | 固定上游提交的 PAL mesh、xacro、配置和许可证来源 |
| `config/joint_mapping.yaml` | 23 个关节、11 条 vCAN、Node ID、模式、单位和标定真源 |
| `config/can_protocol.yaml` | CAN v1 线协议机器可读定义 |
| `include/tiago_can/`, `src/` | 协议、SocketCAN、VirtualMotor 和 Webots gateway |
| `tools/` | 单关节控制、监控和手臂演示程序 |
| `scripts/` | 模型生成、vCAN 创建、Webots/网关启动和 23 节点验证脚本 |
| `tests/` | CTest、pytest、SocketCAN 闭环和底盘真实位移测试 |
| `docs/` | 后端协议、节点映射、模型报告和运行说明 |

## 9. 后端验收命令

静态和单元测试：

```bash
cd /home/kuang/workspace/tiago_can_webots
ctest --test-dir build --output-on-failure
.venv/bin/pytest -q
.venv/bin/python scripts/validate_webots_model.py
```

已有 Webots 和网关时验证 23 节点、总线隔离、head/torso/wheel 往返以及
stop/disable/watchdog：

```bash
./scripts/verify_can_motion.py
```

成功摘要应为：

```text
SUMMARY passed=23 requested=23 isolation_passed=23 failed=0 cleanup_errors=0
```

完整自动启动验证使用 `bash scripts/run_motion_verification.sh`；真实位移另用
`bash tests/test_base_displacement.sh`。自动运动和故障注入脚本只允许 `vcan*`，
不得对 `can0` 执行。

## 10. 安全限制

- 仿真 Node ID、编码器、方向、零偏和接触参数都不是真机标定。
- Robot 已解锁；启动 Webots 前应清空场地，并使用低速命令。
- stop、disable 和单节点 `500 ms` watchdog 继续生效，但不能替代整车级急停。
- 接真实 CAN 前必须重新确认波特率、终端电阻、Node ID、限位、编码器和方向，并
  使用独立急停、机械限位与硬件使能链。
