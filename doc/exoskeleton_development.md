# 外骨骼开发与验证说明

> 本文件是仓库中唯一持续维护的外骨骼文档。内容只覆盖外骨骼本身：USB 串口输入、协议、slot/方向标定、7 自由度机械结构验证器、开发接口和测试方法。
>
> 当前版本：2026-09。凡标记为“待验证”的内容，不能直接作为下游执行器的运动标定。

## 0. 核心约定

外骨骼输入链路分为四步：

```text
USB CDC 串口
    ↓  VID:PID 选择设备
字节流解码：0xAA + payload + XOR + 0x55
    ↓
SDK slot 原始计数 → 弧度
    ↓
穿戴者视角的正方向和 7 自由度机械结构显示
```

数据链路中有以下三种不同概念：

1. **原始计数转弧度**：只使用协议比例，不涉及左右镜像和目标设备。
2. **外骨骼自身的正方向**：以穿戴者站立、双臂自然下垂，并低头观察自己的关节为准。
3. **3D 观察器的显示坐标**：是把第 2 项画到屏幕上的实现约定，不能反过来改变第 2 项的物理含义。

当前代码已经提供串口发现、帧解码、掉线失效保护、8 个 slot 监视器和 7 自由度机械结构 3D 观察器。slot 7 尚未定义，不参与姿态显示和运动解释。

## 1. 文件职责和参考来源

| 文件 | 职责 |
| --- | --- |
| [`src/input/exoskeleton/exoskeleton.hpp`](../src/input/exoskeleton/exoskeleton.hpp) | 配置、状态、解码器和串口输入接口 |
| [`src/input/exoskeleton/exoskeleton.cpp`](../src/input/exoskeleton/exoskeleton.cpp) | 串口线程、设备发现、重连、状态失效和配置加载 |
| [`src/input/exoskeleton/exoskeleton_protocol.hpp`](../src/input/exoskeleton/exoskeleton_protocol.hpp) | 帧常量、字段结构和计数转弧度常量 |
| [`src/input/exoskeleton/exoskeleton_protocol.cpp`](../src/input/exoskeleton/exoskeleton_protocol.cpp) | 帧校验和 payload 字段解析 |
| [`src/input/exoskeleton/exoskeleton_stream_decoder.cpp`](../src/input/exoskeleton/exoskeleton_stream_decoder.cpp) | 拆包、粘包、噪声和多种帧长度重同步 |
| [`tools/exoskeleton_serial.py`](../tools/exoskeleton_serial.py) | Python 工具共用的 VID:PID 串口发现逻辑 |
| [`tools/exoskeleton_joint_monitor.py`](../tools/exoskeleton_joint_monitor.py) | 只显示原始值和弧度，不控制任何执行器 |
| [`tools/exoskeleton_3d_viewer.py`](../tools/exoskeleton_3d_viewer.py) | 7 自由度机械结构的实时/演示 3D 观察器 |
| [`tests/exoskeleton_protocol_test.cpp`](../tests/exoskeleton_protocol_test.cpp) | 协议和解码器测试 |
| [`tests/exoskeleton_runtime_test.cpp`](../tests/exoskeleton_runtime_test.cpp) | 伪终端连接、有效帧、断开和 stale 行为测试 |

[`doc/reference/remote_manipulator_data_reader.py`](reference/remote_manipulator_data_reader.py) 是供应商/历史 SDK 参考实现。它用于核对帧布局和计数转弧度；其中旧的 ROS 回调、摇杆归一化和控制逻辑不是本项目的外骨骼映射规范。

## 2. 串口设备识别

### 2.1 VID:PID 是设备身份

默认设备身份为：

```text
VID = 0x0483
PID = 0x5740
```

C++ 输入层扫描 `/sys/class/tty` 下的 `ttyACM*` 和 `ttyUSB*`，通过 USB 父设备的 `idVendor`/`idProduct` 选择设备。Python 工具使用 `pyserial` 返回的 `vid`/`pid` 做同样的精确匹配。

串口名（例如 `/dev/ttyACM0`）只表示当前一次枚举结果。拔插后它可能变成 `/dev/ttyACM1`，所以不能把串口名写成长期设备身份。

默认要求**恰好一个** VID:PID 匹配设备：

- 没有匹配设备：等待并按重连周期重试；
- 有多个匹配设备：等待并报告歧义，不猜测；
- `match_vid_only: true`：只按 VID 匹配，仅适用于明确知道现场只有一个目标设备的调试场景。

`--port /dev/tty...` 是 Python 工具的临时覆盖选项，适合排查枚举问题，不应替代部署配置。

外骨骼输入层只读串口，不向外骨骼写控制指令。串口打开后启用内核级独占；独占失败、串口断开或读错误都会记录日志并进入重连。串口断开、读错误、重连或停止时，最新状态会清空；没有新的有效帧超过 `stale_timeout_ms` 时，`stateFresh()` 会变成 false，即使缓存对象还存在，消费者也必须把它当成无效输入。校验失败的帧会被丢弃，不会替换上一帧。

### 2.2 当前 C++ 配置接口

仓库提供 `config/exoskeleton.yaml` 作为输入层默认配置。现有
`loadExoskeletonConfig()` 识别以下字段：

| 字段 | 当前要求/默认值 | 当前含义 |
| --- | --- | --- |
| `exoskeleton.serial.device` | 可选，默认空字符串 | 空字符串或 `auto` 走 VID:PID 发现；其他字符串作为显式串口路径 |
| `exoskeleton.serial.usb_vid` | 必填 | USB VID，支持十进制或 `0x` 十六进制 |
| `exoskeleton.serial.usb_pid` | 必填 | USB PID，支持十进制或 `0x` 十六进制 |
| `exoskeleton.serial.match_vid_only` | 必填 | `false` 时匹配 VID+PID；`true` 时只匹配 VID |
| `exoskeleton.serial.baudrate` | 必填 | 串口速率 |
| `exoskeleton.serial.poll_timeout_ms` | 可选，20 ms | 串口 poll 超时 |
| `exoskeleton.serial.reconnect_interval_ms` | 可选，1000 ms | 设备未找到或断开后的重连间隔 |
| `exoskeleton.telemetry.stale_timeout_ms` | 可选，100 ms | `stateFresh()` 判定输入仍新鲜的最长时间 |
| `exoskeleton.telemetry.frame_size` | 可选，`Full` | `auto`、`51`、`91` 或 `131` |

Python 工具默认串口速率为 `2000000`。运行时测试使用伪终端，因此测试中的速率不代表硬件速率。

## 3. 串口协议

### 3.1 完整帧

```text
0xAA | payload | checksum | 0x55
```

- `payload` 长度为 48、88 或 128 字节；
- 完整帧长度分别为 51、91 或 131 字节；
- 多字节字段使用 little-endian；
- `checksum` 是 payload 所有字节的逐字节 XOR；
- 帧头、校验和、帧尾不属于 payload。

解码器支持串口一次读到半帧、多个帧连在一起以及帧前有噪声的情况。固定帧长度模式只接受指定长度；`auto` 模式会在合法帧后记住成功的长度，并在失步时重新寻找帧头和合法校验。

### 3.2 payload 布局

偏移量相对于 payload 起点：

| 偏移 | 长度 | 内容 | 字段 |
| ---: | ---: | --- | --- |
| 0 | 8 | 左摇杆 | `raw_x: int16`、`raw_y: int16`、`key_mask: uint16`、`trigger: int16` |
| 8 | 8 | 右摇杆 | 同上 |
| 16 | 16 | 左臂 | 8 个 little-endian `int16` slot |
| 32 | 16 | 右臂 | 8 个 little-endian `int16` slot |
| 48 | 40 | 躯干 IMU | torso/full 载荷存在 |
| 88 | 40 | 额外/头部 IMU | full 载荷存在 |

因此：

```text
left_arm.slot[i]  偏移 = 16 + 2*i
right_arm.slot[i] 偏移 = 32 + 2*i
```

`i` 的范围是 0～7。SDK 关节编号从 1 开始，SDK 关节编号 `n` 对应 `slot n-1`。

### 3.3 摇杆和按键

输入层保留完整的 16 位 `key_mask`，不在解析层猜测运行模式。当前已知语义：

- bit 0～4：常用按键，协议记录为 active-low；
- bit 5：左右运行 toggle；
- bit 8～12：扩展按键。

把 bit 转换为“按下/释放”时，必须按 active-low 语义处理对应按键；不能把所有按键统一当作 active-high。

## 4. 原始计数转弧度

供应商参考实现和当前 C++ 解析器使用同一公式：

```text
encoder_rad = signed_raw * (2π / 16384)
```

其中 `signed_raw` 是 little-endian 两字节按二补码解释后的 `int16`。

这一步只完成：

```text
编码器原始计数 → 外骨骼协议层的弧度值
```

它不会自动完成：

- 左右臂镜像；
- 正方向反转；
- 自然下垂零位校正；
- 减速比或电机轴变换；
- slot 重排或关节合并。

不要对这一步再次乘以 360、16384 或其他设备的编码器分辨率。外骨骼串口的 `16384` 是协议比例，不等同于下游电机的编码器配置。

## 5. 穿戴者视角的关节定义

### 5.1 基准姿态和术语

基准姿态是：**人体站立，双臂自然下垂**。

表中的方向均采用**穿戴者第一人称视角**：穿戴者低头看自己的肩、肘和腕。

- “向前/向后”沿穿戴者面向的前后方向；
- “身体内侧”指朝身体正中线，“身体外侧”指远离身体正中线；
- “顺时针/逆时针”只表示穿戴者低头观察自己的肘部时看到的旋转方向；
- 屏幕当前从正面、背面或侧面观察，都不改变上面的物理定义。

slot 2 是绕上臂长轴的旋转，slot 4 是绕前臂长轴的旋转。手臂伸直时两根轴可能近似共线，但它们是两个不同的电机和两个独立数据。

slot 3 是肘部屈伸：大臂保持不动，小臂整体从下方向人体面部移动，或反向远离面部；它不是沿小臂长轴的旋转。

slot 5 和 slot 6 位于同一个腕部安装位置，代表腕部的两个自由度；它们不是两个相隔的空间关节。

### 5.2 左臂现场标定表

| SDK 关节编号 | SDK slot | 位置/轴 | 自由度含义 | 穿戴者第一人称正向动作 | 方向记录 |
| ---: | ---: | --- | --- | --- | ---: |
| 1 | 0 | 肩部前后 | 上臂相对躯干的前后自由度 | 上臂向身体后方移动 | +1 |
| 2 | 1 | 肩部侧向 | 上臂相对躯干的侧向自由度 | 上臂向身体内侧移动 | +1 |
| 3 | 2 | 上臂长轴 | 上臂轴向旋转 | 低头观察自己的左肘：向身体外侧旋转，观察为逆时针 | +1 |
| 4 | 3 | 肘部屈伸轴 | 小臂相对大臂的屈伸 | 大臂不动，小臂从下方向面部移动 | +1 |
| 5 | 4 | 前臂长轴 | 前臂轴向旋转 | 低头观察自己的左肘：顺时针旋转 | +1 |
| 6 | 5 | 腕部轴 A | 腕部前后自由度；与 slot 6 共用安装位置 | 腕部向身体后方移动 | +1 |
| 7 | 6 | 腕部轴 B | 腕部左右自由度；与 slot 5 共用安装位置 | 腕部向身体中心移动，观察为顺时针 | +1 |
| 8 | 7 | 尚未定义 | 当前日志为 0.0 | 无已确认动作 | 未定 |

### 5.3 右臂现场标定表

右臂 slot 0 的最终现场记录是“向前”，不是早期原始记录中的“向上”。

| SDK 关节编号 | SDK slot | 位置/轴 | 自由度含义 | 穿戴者第一人称正向动作 | 方向记录 |
| ---: | ---: | --- | --- | --- | ---: |
| 1 | 0 | 肩部前后 | 上臂相对躯干的前后自由度 | 上臂向身体前方移动 | +1 |
| 2 | 1 | 肩部侧向 | 上臂相对躯干的侧向自由度 | 上臂向身体外侧移动 | +1 |
| 3 | 2 | 上臂长轴 | 上臂轴向旋转 | 低头观察自己的右肘：向身体内侧旋转，观察为逆时针 | +1 |
| 4 | 3 | 肘部屈伸轴 | 小臂相对大臂的屈伸 | 大臂不动，小臂从下方向面部移动 | +1 |
| 5 | 4 | 前臂长轴 | 前臂轴向旋转 | 低头观察自己的右肘：顺时针旋转，同时向身体外侧 | +1 |
| 6 | 5 | 腕部轴 A | 腕部前后自由度；与 slot 6 共用安装位置 | 腕部向身体后方移动 | +1 |
| 7 | 6 | 腕部轴 B | 腕部左右自由度；与 slot 5 共用安装位置 | 腕部向身体外侧移动，观察为顺时针 | +1 |
| 8 | 7 | 尚未定义 | 当前日志为 0.0 | 无已确认动作 | 未定 |

这两张表描述的是**外骨骼源侧**的物理语义。表里的 `+1` 是现场方向记录；它不是屏幕坐标轴的符号，也不改变原始计数转弧度的公式。

## 6. 7 自由度机械结构 3D 验证器

### 6.1 结构对应关系

[`tools/exoskeleton_3d_viewer.py`](../tools/exoskeleton_3d_viewer.py) 按实际机械链显示 7 个已定义自由度：

| slot | 机械结构 | 模型处理 |
| ---: | --- | --- |
| 0 | 肩部前后电机 | 肩部第一转轴 |
| 1 | 肩部侧向电机 | 同一肩部串联的第二转轴 |
| 2 | 上臂中部旋转电机 | 绕上臂轴旋转 |
| 3 | 肘部屈伸电机 | 只改变小臂相对大臂的夹角 |
| 4 | 小臂旋转电机 | 绕小臂轴旋转 |
| 5 | 腕部前后电机 | 在腕部节点施加第一个轴 |
| 6 | 腕部左右电机 | 在同一个腕部节点继续施加第二个轴 |
| 7 | 未定义 | 不参与姿态 |

模型按机械串联顺序施加：

```text
slot 0 → slot 1 → slot 2 → slot 3 → slot 4 → slot 5 → slot 6
```

slot 5 和 slot 6 共享一个腕部位置；slot 2 和 slot 4 保持为两个独立旋转轴；slot 3 是小臂相对大臂的屈伸，不会被错误地画成轴向旋转。

### 6.2 观察器坐标和当前符号

观察器世界坐标约定：

```text
+X：穿戴者右方
+Y：穿戴者前方
+Z：向上
```

当前按第 5 节穿戴者视角表实现的“源值 → 观察器显示”符号为：

```text
左臂 slot 0..6：[-1, +1, -1, +1, +1, -1, +1]
右臂 slot 0..6：[+1, +1, -1, +1, +1, -1, +1]
```

这是 3D 显示层在上述世界坐标中的确定实现，不是重新定义现场正方向。旋转浏览器视角不会改变人体前后左右的物理含义。

### 6.3 启动和操作

不接串口的演示模式：

```bash
python3 tools/exoskeleton_3d_viewer.py --demo
```

实时模式默认按 VID:PID 查找：

```bash
python3 tools/exoskeleton_3d_viewer.py
```

常用参数：

```text
--vid 0x0483 --pid 0x5740   修改 USB 身份
--port /dev/ttyACM1         临时指定串口
--baudrate 2000000          修改串口速率
--absolute                  不把首次有效帧作为相对显示基准
--stale-timeout 0.2         超时后停止显示新姿态
--no-browser                只启动服务，不自动打开浏览器
```

交互方式：

- 鼠标拖拽：旋转观察视角；
- 鼠标滚轮：缩放；
- W/A/S/D：观察位置前后左右移动；
- 空格：上升；
- Shift：下降；
- R：重置视角；
- 页面上的全屏按钮：切换全屏；
- 绿色地面：建立站立、前后和上下的空间参照。

实时模式默认是“相对显示”：每次重新连接后的第一帧作为观察器显示基准，便于观察动作方向。这只是显示功能，不是输入层的运行时标定，也不会修改协议弧度；查看绝对弧度时使用 `--absolute`。

## 7. 外骨骼数据的使用规则

### 7.1 原始值、弧度和方向变换

外骨骼模块输出的 `left_arm_joint_rad`/`right_arm_joint_rad` 是协议层弧度。显示层或其他使用方若增加方向、slot 或零位变换，形式为：

```text
source_rad[i] = signed_raw[i] * 2π / 16384
display_or_consumer_rad[j] = sign[j] * source_rad[source_slot[j]] + offset[j]
```

其中 `sign`、`source_slot` 和 `offset` 属于使用方的映射，不属于协议解码器。外骨骼自身的 slot 语义以第 5 节表格为准。

### 7.2 已确认的现场映射

现场标定已经通过 `tools` 下的测试工具确认。输入层不在启动时采集第一帧作为零位，也不执行运行时标定；它始终输出协议原始值和协议弧度。需要反映穿戴者姿态时，由显示层或其他使用方使用已经确认的 slot、方向和固定偏置映射。

`--absolute` 只改变观察器显示方式，不改变底层协议弧度。

### 7.3 独立电机数据

当前输入层保留以下独立数据：

- slot 2 与 slot 4 是两个轴向旋转数据；
- slot 5 与 slot 6 虽然位于同一个腕部安装位置，仍是两个独立数据；
- slot 7 当前未定义，日志中通常为 0.0。

输入层不对这些 slot 求和、平均或重排。

## 8. 测试说明

### 8.1 构建和运行外骨骼测试

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --target exoskeleton_protocol_test exoskeleton_runtime_test
ctest --test-dir build -R '^exoskeleton_'
```

`exoskeleton_protocol` 覆盖：

- 帧头、帧尾和 XOR 校验；
- 51/91/131 三种完整帧长度；
- payload 字段偏移和 little-endian；
- 有符号 slot 以及 `2π/16384` 转换；
- 拆包、粘包、噪声和重新同步。

### 8.2 `exoskeleton_runtime` 验证什么

[`tests/exoskeleton_runtime_test.cpp`](../tests/exoskeleton_runtime_test.cpp) 不需要真实外骨骼、USB 设备或执行器。它使用 POSIX pseudo-terminal 模拟串口：

1. 创建伪终端并取得从端路径；
2. 用显式伪终端路径启动 `Exoskeleton`，配置中仍带有 VID/PID 字段；
3. 从主端写入一帧 131 字节的合法 full 帧；
4. 左 slot 0 写入 `1234`，右 slot 0 写入 `-2345`；
5. 检查线程完成连接、原始值解析正确、弧度转换正确；
6. 关闭伪终端主端；
7. 检查最新状态被清空、连接失效、`stateFresh()` 为 false；
8. 停止线程并清理伪终端。

这个测试验证的是“串口线程和状态生命周期”，不包含人体方向和 3D 几何方向判定；方向表和模型规则见第 5、6 节。

### 8.3 Python 工具的快速检查

```bash
python3 -m py_compile tools/exoskeleton_serial.py tools/exoskeleton_joint_monitor.py tools/exoskeleton_3d_viewer.py
python3 tools/exoskeleton_3d_viewer.py --demo --no-browser
```

演示服务启动后，可在浏览器中确认模型、绿色地面、鼠标旋转、WASD、升降和全屏按钮；关闭服务后不会触碰串口。

## 9. 当前行为和未定义项

- 监视器和实时观察器都会打开外骨骼串口；同时运行时会产生串口占用或竞争，当前代码没有多进程协调。
- 默认严格匹配 VID:PID；匹配到多个设备时，C++ 输入层不选择其中任意一个。
- 断开、读错误、重连和停止会清空最新状态；超过 stale 时间时 `stateFresh()` 变为 false，即使缓存对象还存在。
- 输入层只读串口，不向外骨骼发送电机控制命令。
- slot 7 尚未定义，当前日志虽常为 0.0，但没有已确认的物理含义。
- 3D 观察器用于显示外骨骼自身的空间方向；相对模式的首帧是显示基准，不是人体绝对零位。
- slot 2/4 和 slot 5/6 在输入层分别保留，不求和、不平均、不重排。
