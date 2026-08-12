# ZK Robot

TIAGo 机器人 SocketCAN 驱动实验工程。当前实现从 YAML 加载 CAN 总线与关节参数，完成协议帧处理、编码器/关节单位换算，并向单个电机发送使能和位置指令。

## 当前能力

- SocketCAN 接口的打开、收发与错误处理。
- YAML 格式的多总线、关节、限位和编码器配置。
- CAN 协议编解码与编码器位置换算。
- 电机使能及带速度上限的位置命令。
- TIAGo 八路总线配置样例，详见 [`config/tiago/can/README.md`](config/tiago/can/README.md)。

## 构建

需要支持 SocketCAN 的 Linux、CMake 3.16+、C++17 编译器和 `yaml-cpp`：

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

运行入口为：

```bash
./build/robot
```

## 配置与测试

`main.cpp` 当前读取 `config/tiago/can/left_shoulder.yaml`，选择其中第一个关节并发送 `0.1 rad`、速度上限 `0.1 rad/s` 的位置命令。仓库样例默认使用 `vcan0` 至 `vcan7`；可先创建虚拟 CAN 接口验证配置和报文链路。

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## 安全边界

当前入口会在启动后直接发送电机使能和位置命令，不应在未核对节点 ID、编码器方向、零位、关节限位和急停状态时连接物理 CAN。虚拟总线只能验证软件链路，不能替代真实驱动器的限位、故障和掉线测试。

该工程仍是驱动实验基线，尚未提供完整的多关节调度、反馈闭环、故障恢复和自动化测试套件。
