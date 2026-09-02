# 调试工具说明

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
  只更新数值，内置 legacy 帧解析，默认按 USB VID:PID 自动找串口，便于穿戴状态下逐轴
  标定；不连接执行器。
- `exoskeleton_3d_viewer.py`：通过本地浏览器显示带厚度、遮挡和明暗的低多边形 3D 七自由度机械结构；
  slot 0/1 是肩部前后和侧向电机，slot 2 是大臂中段旋转，slot 3 是肘部屈曲，slot 4 是小臂旋转，
  slot 5/6 是腕部两个电机，slot 7 仅显示不参与姿态；每个槽位直接按弧度和现场正方向符号驱动对应
  转轴。支持 `--demo` 离线演示，不连接执行器；支持鼠标拖动旋转视角、滚轮缩放、WASD 平移、
  空格上升、Shift 下降和全屏，并绘制绿色网格地面与黄色面向箭头辅助判断。

外骨骼联调时，下面两个 Python 读取器二选一；切换到另一个之前先退出当前程序：

```bash
python3 tools/exoskeleton_joint_monitor.py
python3 tools/exoskeleton_3d_viewer.py
```

两个工具默认匹配 `VID:PID=0x0483:0x5740`，也可以用 `--vid` 和 `--pid` 覆盖；只有
需要临时绕过自动发现时才使用 `--port /dev/ttyACM...`。

需要使用 C++ 监视器时，也要先退出 Python 读取器：

```bash
cmake --build build --target exoskeleton_monitor -j2
./build/exoskeleton_monitor /path/to/exoskeleton-config.yaml
```

当前工作树不提供固定的外骨骼运行配置文件，配置格式正在重新规划；上面的路径需要替换为实际配置。
现有 loader 中，`exoskeleton.serial.device` 为空或为 `auto` 时按 `usb_vid`、`usb_pid` 发现设备，
不应把 `/dev/ttyACM0` 写成长期身份。

没有外骨骼时可以先看离线演示：

```bash
python3 tools/exoskeleton_3d_viewer.py --demo
```

官方读取器快照、协议布局和弧度换算见 `doc/exoskeleton_development.md`。
当前 Python 监视器和 C++ 实时路径都不依赖完整厂商 SDK；需要 QnTP 校准、无线、
触觉、DFU 或厂商 3D 工具时，应向厂商索取对应版本的独立工具。
完成 8 个 slot（其中 slot 7 未定义）的现场逐轴标定后，才允许把外骨骼数据交给任何实体控制程序。

方向工具的运动命令经过正式 `Joint` 限位检查，但刻意使用恒等坐标换算，继续记录
“电机输出角正增量”对应的实体运动方向。自然下垂回零属于电机零点与边界恢复流程，
不会整体迁入使用模型坐标的 Arm；Arm 检查发现当前位置越界时会要求先运行回零工具。

回零、方向和整臂工具默认排除腰部、折叠机构和傲意手；整机组合工具按流程测试
双臂、头部和灵巧手。独立灵巧手工具只使用傲意手适配器。
编译及操作前必须阅读 `doc/` 中对应文档，关闭其他 CAN 控制进程，并保证物理急停
可以立即触达。
