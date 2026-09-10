# 调试工具说明

`tools/` 按功能分为两个目录：

- `tools/ti5/`：TI5 T170C 本体调试工具。
- `tools/exoskeleton/`：外骨骼数据读取、观察和遥操作工具。

这些程序可能打开实体 CAN、串口或让机器人运动，不由 `ctest` 自动运行。
编译和实机操作前应关闭其他控制程序，并保证物理急停可以立即触达。

## TI5

- [`ti5/ti5_joint_cli.cpp`](ti5/ti5_joint_cli.cpp)：交互式关节控制台。
  通过 CAN 自动发现腰部、头部和双臂总线，主菜单可进入头部、左臂、右臂、
  左手、右手和腰部；使用数字键选择关节，用方向键或 `+`/`-` 调整目标。
  头部、双臂和腰部使用弧度，灵巧手使用 raw 位置单位。
- [`ti5/ti5_zero_home.cpp`](ti5/ti5_zero_home.cpp)：头部和双臂 17 轴回零及
  特定点工具。支持电机角零点回零、双臂受控缓降后 STOP、记录当前位置特定点、
  以及运行已记录特定点；不控制腰部、折叠机构和灵巧手。

TI5 工具的用途、参数、覆盖范围和操作入口见
[`doc/ti5测试代码合集.md`](../doc/ti5测试代码合集.md)。

## 外骨骼

- [`exoskeleton/exoskeleton_monitor.cpp`](exoskeleton/exoskeleton_monitor.cpp)：
  只读显示外骨骼关节、手柄和 IMU 遥测，不连接机器人执行器。
- [`exoskeleton/exoskeleton_joint_monitor.py`](exoskeleton/exoskeleton_joint_monitor.py)：
  在终端显示外骨骼 8 个编码器槽位的原始值和弧度，不控制执行器。
- [`exoskeleton/exoskeleton_3d_viewer.py`](exoskeleton/exoskeleton_3d_viewer.py)：
  通过浏览器显示外骨骼实时或离线演示的 3D 诊断视图。
- [`exoskeleton/exoskeleton_tiago_teleop.cpp`](exoskeleton/exoskeleton_tiago_teleop.cpp)：
  将外骨骼输入映射到 TIAGo；只有完成标定、映射和安全确认后才能发送控制目标。
- [`exoskeleton/exoskeleton_serial.py`](exoskeleton/exoskeleton_serial.py)：
  Python 工具共用的 USB VID:PID 串口发现逻辑。

外骨骼读取器：

```bash
python3 tools/exoskeleton/exoskeleton_joint_monitor.py
python3 tools/exoskeleton/exoskeleton_3d_viewer.py
```

没有外骨骼时可运行离线演示：

```bash
python3 tools/exoskeleton/exoskeleton_3d_viewer.py --demo
```

外骨骼协议、标定和安全边界见
[`doc/exoskeleton_development.md`](../doc/exoskeleton_development.md)。
