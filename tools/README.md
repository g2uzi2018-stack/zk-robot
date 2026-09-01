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
- `exoskeleton_joint_monitor.py`：固定布局显示左右臂 8 个官方编码器槽位，
  只更新数值，内置 legacy 帧解析，便于穿戴状态下逐轴标定；不连接 TIAGo。

外骨骼联调建议先运行：

```bash
python3 tools/exoskeleton_joint_monitor.py --port /dev/ttyACM0
cmake --build build --target exoskeleton_monitor exoskeleton_tiago_teleop -j2
./build/exoskeleton_monitor config/exoskeleton.yaml
```

官方读取器快照、协议布局和弧度换算见 `doc/exoskeleton_development.md`。
当前 Python 监视器和 C++ 实时路径都不依赖完整厂商 SDK；需要 QnTP 校准、无线、
触觉、DFU 或厂商 3D 工具时，应向厂商索取对应版本的独立工具。
完成 8 槽到 TIAGo 7 关节的现场逐轴标定后，才允许进入实体控制。

方向工具的运动命令经过正式 `Joint` 限位检查，但刻意使用恒等坐标换算，继续记录
“电机输出角正增量”对应的实体运动方向。自然下垂回零属于电机零点与边界恢复流程，
不会整体迁入使用模型坐标的 Arm；Arm 检查发现当前位置越界时会要求先运行回零工具。

回零、方向和整臂工具默认排除腰部、折叠机构和傲意手；整机组合工具按流程测试
双臂、头部和灵巧手。独立灵巧手工具只使用傲意手适配器。
编译及操作前必须阅读 `doc/` 中对应文档，关闭其他 CAN 控制进程，并保证物理急停
可以立即触达。
