# ZK Robot

机器人 C++/SocketCAN 控制工程。仓库保留现有 TIAGo 实现，并按照相同的分层方式逐步
接入 TI5 T170C。

## 代码分层

```text
src/can/             与机器人型号无关的 Linux SocketCAN 公共层
src/input/exoskeleton/ Qnbot 外骨骼串口输入、legacy 遥测解析和重连
src/tiago/           现有 TIAGo 分层实现
src/ti5/config/      TI5 拓扑和 CAN 配置读取
src/ti5/can/         TI5 协议、节点发现、总线反馈和总线健康状态
src/ti5/motor/       TI5 单个实体电机
src/ti5/joint/       TI5 单个物理关节、坐标换算和目标限位
src/ti5/hand/        傲意灵巧手独立协议
tools/               需要明确操作、可能连接实体机器人的调试工具
tests/               不连接实体机器人的自动测试
qnbot-exoskeleton-SDK/ 厂商 Python SDK 原始快照，不参与 CMake 构建
```

后续 TI5 的 `arm/head/controller/executor` 将参考 `src/tiago/` 的现有层级继续
向上增加。多关节同步、控制周期和轨迹插值不属于 `Joint` 或 `CanMotor`。

## TI5 Motor 阶段能力

- 识别本体四通道 USB-CAN 适配器，并将逻辑总线映射到实际 `canX`。
- 使用 `0x08` 只读发现节点并读取当前位置。
- 读取驱动器 `0x1A/0x1B` 目标位置范围。
- 使用 `0x41/0x44` 查询和发送 Position CSP，解析位置、速度和电流反馈。
- 查询 `0x03` 运行模式和 `0x0A` 原始故障位。
- 区分普通位置查询和真正 CSP 反馈的新鲜度序号。
- 使用 SocketCAN 节点过滤，并记录错误警告、错误被动、总线关闭和自动恢复事件。
- 记录发送失败次数；CAN 层只报告健康状态，不会自动恢复机器人运动。

当前 Motor 层只开放已确认的双编码器 Position CSP，不实现猜测性的 `0x01` 使能、
PT、电流/速度控制、参数写入、抱闸或零位写入。

## TI5 Joint 阶段能力

- 一个 `Joint` 只拥有一个 `CanMotor`，继续共享所属逻辑总线的 `CanBus`。
- 按 `motor_rad = joint_rad * direction + offset_rad` 转换模型关节角和电机输出角。
- 统一加载 `safety.yaml` 的主机软件位置限位，以及 `kinematics.yaml` 的方向和零偏。
- 下发 Position CSP 前同时检查主机软件限位和驱动器 `0x1A/0x1B` 目标范围。
- 驱动器目标范围未成功读取时拒绝发送运动目标。
- 将位置和速度反馈转换为关节坐标；电流、运行模式、故障和反馈新鲜度继续来自 Motor。

Joint 不会把当前位置越出驱动器目标范围直接当成无效反馈，也不会自动执行肩横滚边界
恢复。自然下垂接管、边界恢复轨迹和失败后的多关节处理属于后续启动控制。

## 构建与自动测试

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

这些自动测试使用内存中的模拟 CAN 传输，不打开实体 CAN 接口。

## 外骨骼控制

厂商资料包的主入口是 `qnbot-exoskeleton-SDK/qnbot_sdk_v1.2/qnbot_sdk.py`：
`QnbotClient`/`QnbotStreamParser` 负责 QnTP 请求响应和 51/91/131 字节 legacy
遥测流。仓库中的 C++ `exoskeleton_input` 按该 SDK 的 legacy 数据模型提供确定性的
实时输入；厂商 Python SDK 继续用于版本/能力查询、手柄和 IMU 校准、Wireless、
Haptics 及 DFU。旧的 `remote_manipulator_data_reader.py` 不作为 C++ 控制链路。

先用厂商 SDK 检查设备，再做只读监测：

```bash
python3 tools/qnbot_vendor_probe.py \
  --port /dev/exoskeleton \
  --show-topology \
  --show-calibration
cmake --build build --target exoskeleton_monitor exoskeleton_tiago_teleop -j2
./build/exoskeleton_monitor config/exoskeleton.yaml
```

`--show-calibration` 输出的 `Hand.GetCalibParams` 数值用于填写
`config/exoskeleton_tiago_teleop.yaml`。厂商 SDK 不定义 8 个编码器槽位对应的
人体/机器人关节，因此还必须现场逐轴标定 `retargeting`。在
`handset_calibration.verified` 和 `retargeting.verified` 都改为 `true` 前，TIAGo
遥操作的 `--confirm` 会停在等待状态，不能向实体机器人下发目标。

确认外骨骼模式、方向、零偏、限位和急停均已验证后，才运行
`exoskeleton_tiago_teleop --confirm`。该工具控制的是 TIAGo 双臂、双夹爪和底盘；
目前没有把外骨骼 IMU 自动映射到 TIAGo 头部/腰部，也没有默认打开任何实体运动。

根目录的 `robot` 是一个安全的配置 smoke test，只加载并检查 TI5 配置，不打开 CAN：

```bash
cmake --build build --target robot -j2
./build/robot
```

## 实机调试工具

这些工具不参与默认构建，必须显式编译：

```bash
cmake --build build --target ti5_zero_home ti5_direction_test ti5_full_check -j2
./build/tools/ti5_zero_home --dry-run
./build/tools/ti5_direction_test --help
./build/tools/ti5_full_check --dry-run
```

- `ti5_zero_home`：头部和双臂回零、保持或受控停止菜单。
- `ti5_direction_test`：头部和双臂逐轴小幅方向确认。
- `ti5_full_check`：双臂、头部和左右灵巧手的整机组合小幅往返测试。

实机操作方法和安全约束见 `doc/TI5_T170C_一键回零测试.md`、
`doc/ti5_motor_direction_record.md`。腰部和折叠机构的抱闸、停止及掉线行为尚未完成验证，
不属于当前 Motor 阶段的运动范围。
