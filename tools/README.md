# TI5 实机调试工具

本目录中的程序可能打开实体 CAN 接口或让机器人运动，不由 `ctest` 自动运行。

- `ti5_zero_home.cpp`：头部和双臂回零、保持和受控停止菜单。
- `ti5_direction_test.cpp`：头部和双臂逐轴小幅方向确认。
- `ti5_arm_check.cpp`：基于正式 Arm 模型坐标完成整臂读取、当前位置保持、
  单轴小幅往返和 STOP 状态确认；当前位置超出驱动器目标范围时拒绝运动，
  要求先运行回零工具。
- `ti5_hand_check.cpp`：只初始化傲意灵巧手 CAN，支持左手、右手或双手的
  状态读取，以及六通道或指定单通道的小幅往返；不会初始化双臂和头部。
- `ti5_full_check.cpp`：整机组合测试，按顺序小幅测试双臂、头部和左右灵巧手；
  需要显式构建和现场确认，默认不参与普通构建。
- `exoskeleton_monitor.cpp`：只读打开外骨骼并显示遥测，不打开机器人 CAN。
- `exoskeleton_tiago_teleop.cpp`：将外骨骼双臂、扳机和左手柄映射到 TIAGo；
  默认排除在普通构建之外，必须显式构建并传入 `--confirm`。
- `qnbot_vendor_probe.py`：调用厂商 `QnbotClient` 做版本、能力、拓扑、8 槽 legacy
  遥测和 `Hand.GetCalibParams` 检查；
  与 C++ 监测器或遥操作程序不能同时占用串口。

外骨骼联调建议先运行：

```bash
python3 tools/qnbot_vendor_probe.py \
  --port /dev/exoskeleton \
  --show-topology \
  --show-calibration
cmake --build build --target exoskeleton_monitor exoskeleton_tiago_teleop -j2
./build/exoskeleton_monitor config/exoskeleton.yaml
```

厂商 SDK 目录 `qnbot-exoskeleton-SDK/` 保持原样作为设备协议和维护工具的来源，
不直接加入 CMake；C++ 实时路径只复现其中 legacy 遥测解析，不把 Python 回调线程
放进机器人控制周期。需要 QnTP 校准、无线、触觉或 DFU 时使用厂商 CLI/探测脚本。
将探测器打印的 `Hand.GetCalibParams` 填入遥操作 YAML，并完成 8 槽到 TIAGo 7 关节
的现场逐轴标定；只有 `handset_calibration.verified=true` 和
`retargeting.verified=true` 同时成立时，遥操作才会进入实体控制。

方向工具的运动命令经过正式 `Joint` 限位检查，但刻意使用恒等坐标换算，继续记录
“电机输出角正增量”对应的实体运动方向。自然下垂回零属于电机零点与边界恢复流程，
不会整体迁入使用模型坐标的 Arm；Arm 检查发现当前位置越界时会要求先运行回零工具。

回零、方向和整臂工具默认排除腰部、折叠机构和傲意手；整机组合工具按流程测试
双臂、头部和灵巧手。独立灵巧手工具只使用傲意手适配器。
编译及操作前必须阅读 `doc/` 中对应文档，关闭其他 CAN 控制进程，并保证物理急停
可以立即触达。
