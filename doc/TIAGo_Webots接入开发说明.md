# TIAGo Webots 接入开发说明

## 1. 范围与当前结论

本文是 `zk_robot` 对接本机 TIAGo++ Webots/SocketCAN 仿真的唯一入口文档。
只记录开发接口、启动方式、安全约束和验收标准，不保存密码、GUI 操作、
历史交接记录、外观配色或完整机械参数。

仿真仓库与业务仓库均在本机：

```text
/home/kuang/workspace/tiago_can_webots   Webots 模型、CAN 网关与验证工具
/home/kuang/workspace/zk_robot           控制侧 CAN 驱动与业务代码
```

当前状态：

- Webots 模型共有 `23` 个主动关节，每个都已有同名 Motor 和
  `<joint>_sensor` PositionSensor。
- 网关当前只映射 `18` 个双臂/夹爪关节。
- 未接入的 `5` 个关节是两个轮子、躯干升降和两个头部关节。
- 目标是把全部 `23` 个主动关节接入网关。本文将“当前契约”和
  “23 节点目标”明确分开，未实现的内容不视为现网能力。

## 2. 系统边界与权威数据

```text
zk_robot
  关节目标 / 速度上限 / 状态处理
                    |
                    v
          Linux SocketCAN (vcanN)
                    |
                    v
           can_motor_gateway
                    |
                    v
      Webots Motor + PositionSensor
```

`can_motor_gateway` 是唯一调用 Webots Controller API 的程序。`zk_robot` 只处理
SocketCAN 帧，不应依赖 Webots 进程、设备对象或 PROTO 内部结构。

开发时以以下文件为准：

| 数据 | 权威文件 |
|---|---|
| Webots 场景 | `tiago_can_webots/worlds/tiago_dual_can.wbt` |
| Motor、PositionSensor、硬限位和初始值 | `tiago_can_webots/protos/TiagoDual.proto` |
| 关节类型、软限位和机械链 | `tiago_can_webots/generated/tiago_dual.urdf` |
| 当前 CAN 总线、Node ID 和设备映射 | `tiago_can_webots/config/joint_mapping.yaml` |
| 线协议 | `tiago_can_webots/config/can_protocol.yaml` |
| 控制侧工程限位和标定 | `zk_robot/config/tiago/can/*.yaml` |

不在两个仓库间手工维护两套 Node ID 或总线表。修改映射时必须同步生成或
校验控制侧配置。

## 3. 本机启动方式

### 3.1 构建仿真网关

```bash
cd /home/kuang/workspace/tiago_can_webots
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
.venv/bin/pytest -q
python3 scripts/validate_webots_model.py
```

### 3.2 分终端启动

终端 1：确保 vCAN 存在，然后打开 Webots。

```bash
cd /home/kuang/workspace/tiago_can_webots
bash scripts/setup_vcan.sh
bash scripts/launch_webots.sh
```

如果 vCAN 已由 systemd 服务创建，可跳过 `setup_vcan.sh`。当前脚本只创建
`vcan0..vcan7`。

终端 2：把网关连接到已运行的 `TIAGoDual` Webots Robot。

```bash
cd /home/kuang/workspace/tiago_can_webots
./scripts/run_gateway.sh
```

`run_gateway.sh` 是日常开发的正式网关入口。它：

- 使用 `/usr/local/webots/webots-controller`；
- 连接 Robot 名 `TIAGoDual`；
- 启动 `build/can_motor_gateway`；
- 加载 `config/joint_mapping.yaml`；
- 不创建 vCAN，不启动 Webots，也不发送任何运动目标。

不要在已经运行 Webots/网关时再执行 `scripts/run_demo.sh`；该脚本会自行
启动和回收一组新进程。

### 3.3 无运动查询

```bash
cd /home/kuang/workspace/tiago_can_webots
build/can_joint_control \
  --mapping config/joint_mapping.yaml \
  --urdf generated/tiago_dual.urdf \
  --can-interface vcan0 \
  --joint arm_left_1_joint \
  --query
```

`query` 只读取反馈，不使能电机，不改变目标，也不刷新 `500 ms` 运动看门狗。
网关连接时会让全部已映射 Motor 保持 disabled 并撤销力/力矩，因此受重力影响的
关节可能从 PROTO 初始姿态沉降；运行中的传感器位置不等于初始值并非异常。

## 4. 当前 18 节点契约

| vCAN | 关节 | Node | 命令 ID | 反馈 ID | 单位 |
|---|---|---:|---:|---:|---|
| `vcan0` | `arm_left_1_joint` | `0x01` | `0x101` | `0x181` | rad |
| `vcan0` | `arm_left_2_joint` | `0x02` | `0x102` | `0x182` | rad |
| `vcan1` | `arm_left_3_joint` | `0x03` | `0x103` | `0x183` | rad |
| `vcan1` | `arm_left_4_joint` | `0x04` | `0x104` | `0x184` | rad |
| `vcan2` | `arm_left_5_joint` | `0x05` | `0x105` | `0x185` | rad |
| `vcan2` | `arm_left_6_joint` | `0x06` | `0x106` | `0x186` | rad |
| `vcan2` | `arm_left_7_joint` | `0x07` | `0x107` | `0x187` | rad |
| `vcan6` | `gripper_left_right_finger_joint` | `0x08` | `0x108` | `0x188` | m |
| `vcan6` | `gripper_left_left_finger_joint` | `0x09` | `0x109` | `0x189` | m |
| `vcan3` | `arm_right_1_joint` | `0x11` | `0x111` | `0x191` | rad |
| `vcan3` | `arm_right_2_joint` | `0x12` | `0x112` | `0x192` | rad |
| `vcan4` | `arm_right_3_joint` | `0x13` | `0x113` | `0x193` | rad |
| `vcan4` | `arm_right_4_joint` | `0x14` | `0x114` | `0x194` | rad |
| `vcan5` | `arm_right_5_joint` | `0x15` | `0x115` | `0x195` | rad |
| `vcan5` | `arm_right_6_joint` | `0x16` | `0x116` | `0x196` | rad |
| `vcan5` | `arm_right_7_joint` | `0x17` | `0x117` | `0x197` | rad |
| `vcan7` | `gripper_right_right_finger_joint` | `0x18` | `0x118` | `0x198` | m |
| `vcan7` | `gripper_right_left_finger_joint` | `0x19` | `0x119` | `0x199` | m |

状态 ID 统一为 `0x200 + node_id`，但 CAN v1 不发送独立状态帧；它仅为后续协议
保留。启用、故障和超时状态都在普通反馈帧中。
当前 PROTO 的双臂 14 轴初始值均为 `0 rad`，四个夹爪手指初始值均为 `0.01 m`。

## 5. CAN v1 线协议

所有帧为 Classical CAN 2.0、11 位标准帧、8 字节数据、小端序。Node ID
有效范围为 `1..127`。

### 5.1 命令帧

```text
CAN ID = 0x100 + node_id

byte 0..3  int32   位置 counts，或速度模式的 counts/s
byte 4..5  uint16  最大速度 counts/s
byte 6     uint8   命令码
byte 7     uint8   flags，当前必须为 0
```

| 命令码 | 含义 |
|---:|---|
| `0x01` | enable |
| `0x02` | disable |
| `0x03` | clear fault，清除后仍为 disabled |
| `0x04` | set zero，只修改本次运行的零点 |
| `0x10` | position |
| `0x11` | velocity |
| `0x12` | stop，保持当前位置但不取消 enable |
| `0x20` | query status |
| `0x21` | query position |
| `0x22` | query velocity |

### 5.2 反馈帧

```text
CAN ID = 0x180 + node_id

byte 0..3  int32  位置 counts
byte 4..5  int16  速度 counts/s，超出线宽时饱和
byte 6 bit0       enabled
byte 6 bit1       faulted
byte 6 bit2       timed_out
byte 7            fault code：0=无故障，1=限位故障
```

网关默认每 `20 ms` 发布一次反馈，query 命令会立即触发一次反馈。

### 5.3 安全时序

推荐控制顺序：

```text
query -> clear fault -> enable
      -> 每 100 ms 刷新 position/velocity 命令
      -> 检查反馈状态和跟踪误差
      -> stop -> disable
```

- 电机启动时为 disabled；disabled 或 faulted 状态会拒绝运动命令。
- 位置或速度越限会锁存故障并禁用电机。
- 运动命令超过 `500 ms` 未刷新时，网关进入 Hold 并置 `timed_out=1`。
- query 不刷新看门狗。
- 程序异常或退出时，先撤销全部电机力/力矩，再尝试发送 disabled 反馈。

## 6. 单位、标定与限位

### 6.1 旋转关节

当前仿真默认值为：

```text
counts_per_motor_revolution = 4096
gear_ratio = 1.0
direction = +1
zero_offset = 0
```

```text
counts = round((q - q0) * direction * gear_ratio * 4096 / (2*pi))
q = q0 + counts * 2*pi / (direction * gear_ratio * 4096)
```

这些值是 `simulation_default`，不是 PAL 真机标定。

### 6.2 直线关节

`zk_robot` 对夹爪使用显式线性标定：

```text
counts_per_meter = 4096 / (2*pi) = 651.8986469044033
```

它与当前 Webots 网关把米制位置代入现有 4096-count 公式的线上数值一致。
但网关内部仍使用 `*_rad` 命名和 `counts_per_motor_revolution` 字段表达直线关节。
扩展躯干升降前，应在映射中增加明确的 `joint_type/unit`，并对直线关节使用
`counts_per_meter`，不再依赖隐式解释。这不需改变 8 字节线协议。

### 6.3 限位边界

左右手臂使用相同参数：

| 关节 | 硬限位 rad | URDF 软限位 rad | Webots 最大速度 rad/s |
|---|---:|---:|---:|
| J1 | `-1.178097..1.570796` | `-1.108097..1.500796` | `1.95` |
| J2 | `-1.178097..1.570796` | `-1.108097..1.500796` | `1.95` |
| J3 | `-0.785398..3.926991` | `-0.715398..3.856991` | `2.35` |
| J4 | `-0.392699..2.356194` | `-0.322699..2.286194` | `2.35` |
| J5 | `-2.094395..2.094395` | `-2.074395..2.074395` | `1.95` |
| J6 | `-1.570796..1.570796` | `-1.550796..1.550796` | `1.76` |
| J7 | `-2.094395..2.094395` | `-2.074395..2.074395` | `1.76` |

夹爪单指硬限位为 `0..0.045 m`，URDF 软限位为 `0.001..0.044 m`，Webots 最大
速度为 `0.05 m/s`。

- Webots 网关从 Motor 读取并执行 URDF 硬限位。
- 业务轨迹应使用 URDF `safety_controller` 软限位或更保守的工程限位。
- `zk_robot` 当前对所有手臂关节使用 `10 deg/s` 工程速度上限，低于
  Webots Motor 的最大速度。
- `zk_robot` 对 J5-J7 的位置范围比 URDF 软限位还多预留了 `0.05 rad`；
  这是更保守的工程范围，不应误写成 PAL 原始软限位。

## 7. 23 节点目标

### 7.1 未接入关节的模型参数

| 关节 | 类型 | 初始值 | 硬限位 | URDF 软限位 | 最大速度 | 最大力/力矩 |
|---|---|---:|---:|---:|---:|---:|
| `wheel_right_joint` | continuous | `0 rad` | 无 | 无 | `100 rad/s` | `6 N*m` |
| `wheel_left_joint` | continuous | `0 rad` | 无 | 无 | `100 rad/s` | `6 N*m` |
| `torso_lift_joint` | prismatic | `0.15 m` | `0..0.35 m` | `0..0.35 m` | `0.07 m/s` | `2000 N` |
| `head_1_joint` | revolute | `0 rad` | `-1.308997..1.308997 rad` | `-1.238997..1.238997 rad` | `3 rad/s` | `5.197 N*m` |
| `head_2_joint` | revolute | `0.1 rad` | `-1.047198..0.785398 rad` | `-0.977198..0.715398 rad` | `3 rad/s` | `2.77 N*m` |

TIAGo 这个“腰部”是 `torso_lift_joint` 竖直直线升降关节，不是绕竖直轴旋转的
yaw 关节。

### 7.2 建议的新节点与总线

下表是待实现目标，尚未写入当前 `joint_mapping.yaml`：

| vCAN | 关节 | Node | 命令 ID | 反馈 ID | 主要模式 |
|---|---|---:|---:|---:|---|
| `vcan8` | `wheel_right_joint` | `0x0A` | `0x10A` | `0x18A` | velocity |
| `vcan8` | `wheel_left_joint` | `0x0B` | `0x10B` | `0x18B` | velocity |
| `vcan9` | `torso_lift_joint` | `0x0C` | `0x10C` | `0x18C` | position |
| `vcan9` | `head_1_joint` | `0x0D` | `0x10D` | `0x18D` | position |
| `vcan9` | `head_2_joint` | `0x0E` | `0x10E` | `0x18E` | position |

这个分配使用现有左侧 `0x01..0x09` 和右侧 `0x11..0x19` 之间的空闲 Node ID，
并把底盘与上身分为两个独立故障域。如果之后有真实 CAN 硬件拓扑，应在实现前
用真实总线规划替换 `vcan8/vcan9`。

### 7.3 实现 23 节点必须同时完成的修改

1. 扩展 `extract_robot_data.py` 和 `joint_mapping.yaml`，让 23 个主动关节全部且只出现一次。
2. 扩展 `setup_vcan.sh` 和 systemd vCAN 服务，创建 `vcan8` 与 `vcan9`。
3. 在网关映射中明确关节类型、单位和旋转/直线标定。
4. 修改 `WebotsGateway::motorLimits()` 和 `VirtualMotor`，正确表示无位置限位的
   continuous 轮子；不能把 Webots continuous Motor 的 `0/0` 默认值当成非法限位。
5. 轮子只接受 velocity/stop/disable 为主的控制，不对 continuous 关节发送普通
   有限位位置命令。
6. 如果验收目标包含底盘位移，将 Robot 的 `locked TRUE` 改为可配置/解锁，并
   同步修改模型转换脚本和模型测试。只接入轮子 Motor 但保持 `locked TRUE`
   时，轮子可转但机器人不会行驶。
7. 为 `can_joint_control` 增加米制位置和旋转速度命令，并把 18 关节演示/验证
   拆成手臂夹爪、躯干头部和底盘三类安全测试。
8. 扩展 `zk_robot` 的协议层，实现 `0x11` velocity 命令；再增加 Base、Torso、
   Head 和 Gripper 设备抽象。当前 `Arm` 只管理单侧 7 关节和 3 条手臂总线。
9. 对左右轮命令实施同一控制周期的成对更新。CAN v1 的两次 socket 写入不是
   原子事务，上层必须定义单轮丢帧或一侧总线故障时的整车停车策略。

## 8. `zk_robot` 当前接入边界

`zk_robot/config/tiago/can/` 已有与当前 18 节点一致的八路 YAML，关节名、
vCAN 和 Node ID 相符。但当前高层代码仍有以下边界：

- `Arm` 只表示单侧 7 自由度手臂，不包含夹爪、头部、躯干或底盘。
- `CanMotor` 已实现 enable、disable、clear fault、stop、query status 和 position，
  尚未实现 velocity、set zero、query position 和 query velocity。
- 当前入口程序会直接使能并连续发送左臂位置目标。未检查目标、当前姿态和
  周边环境前，不要运行 `zk_robot/build/robot`。
- 仓库尚无自动化测试套件，协议和配置修改不能只依赖人工运动验证。

## 9. 23 节点验收标准

实现完成后必须同时满足：

1. 生成器、`joint_mapping.yaml`、PROTO 和 URDF 对 23 个主动关节的名称、类型和
   Motor/PositionSensor 一致。
2. CTest、Pytest 和 `validate_webots_model.py` 全部通过。
3. 23/23 节点都能通过正确 vCAN 查询到 PositionSensor 反馈；把同一 Node 发到
   错误总线时不得控制该关节。
4. 14 个手臂关节、4 个夹爪关节、躯干和 2 个头部关节分别完成低速小位移
   往返验收，反馈方向和单位正确。
5. 左右轮分别完成正、反向低速 velocity 验收；解锁底盘后再验证直行、原地转向和
   stop 后的底盘停止。
6. 所有节点通过 disabled 拒绝、越限锁存、`500 ms` 超时、stop、disable 和
   SIGTERM 安全清理测试。
7. 任一测试失败或中断后，所有参与节点最终均为 `enabled=0`，且没有遗留的
   Webots/网关/控制工具进程。

## 10. 安全边界

- 本文的节点、编码器和 vCAN 配置是仿真契约，不是真机标定。
- 自动运动、越限注入和底盘测试只允许在 `vcan*` 上执行。
- 切换到 `can0` 前必须重新标定 Node ID、波特率、编码器、方向和零偏，并使用
  独立急停、机械限位和硬件使能链。
- 轮子控制必须有整车级 watchdog 和一侧故障时的成对停车策略；单节点
  `500 ms` 看门狗不能替代底盘安全控制。
