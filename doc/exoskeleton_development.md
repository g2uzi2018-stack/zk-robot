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

必须区分以下三种概念：

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

外骨骼输入层只读串口，不向外骨骼写控制指令。串口断开、读错误、重连或停止时，最新状态会清空；没有新的有效帧超过 `stale_timeout_ms` 时，`stateFresh()` 会变成 false，即使缓存对象还存在，消费者也必须把它当成无效输入。校验失败的帧会被丢弃，不会替换上一帧。

### 2.2 C++ 配置

C++ 程序默认读取 `config/exoskeleton.yaml`。最小配置如下：

```yaml
exoskeleton:
  serial:
    device: auto
    usb_vid: "0x0483"
    usb_pid: "0x5740"
    match_vid_only: false
    baudrate: 2000000
    poll_interval_ms: 5
    stale_timeout_ms: 100
    reconnect_interval_ms: 500
    frame_size: auto
```

说明：

- `device: auto` 启用 VID:PID 自动发现；
- `device: /dev/ttyACM1` 可用于临时显式指定，但会失去自动换名能力；
- `usb_vid`、`usb_pid` 支持十进制或 `0x` 十六进制字符串；
- `frame_size` 支持 `auto`、`51`、`91`、`131`，推荐 `auto`；
- `stale_timeout_ms` 是上层允许使用最后一帧的最长时间，必须根据设备发送周期设置，不能无限等待旧数据。

Python 工具默认串口速率为 `2000000`。C++ 配置中的 baudrate 必须与设备实际配置一致；运行时测试使用伪终端，因此测试中的速率不代表硬件速率。

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
| 0 | 8 | 左摇杆 | `raw_x: int16`、`raw_y: int16`、`trigger: int16`、`key_mask: uint16` |
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

### 5.1 基准姿态和观察视角

基准姿态是：**人体站立，双臂自然下垂**。

所有“向前、向后、向身体内侧/外侧、顺时针/逆时针”的描述，都采用**穿戴者第一人称视角**：穿戴者低头看自己的肩、肘和腕。顺时针/逆时针不是站在人体对面的观察者视角，也不是程序窗口当前的屏幕视角。

slot 2 和 slot 4 的“顺时针/逆时针”具体含义是：穿戴者低头观察自己的肘部和前臂，看到对应轴绕手臂长度方向转动的方向。手臂伸直时，两根轴在几何上可能近似共线，但它们仍然是两个不同的电机和两个独立数据，不能在输入层合并。

slot 3 是肘部前后/屈伸自由度：大臂保持不动，小臂整体从下方向人体面部移动，或反向远离面部。它不是沿小臂长度方向的旋转。

slot 5 和 slot 6 位于同一个腕部位置，只代表腕部的两个自由度。外骨骼为了提供腕部两个方向的操作而使用两个电机；它们不是两个相隔的腕关节。

### 5.2 左臂现场标定表

| SDK 关节编号 | SDK slot | 物理关节描述 | 穿戴者视角下的正方向 | 方向记录 |
| ---: | ---: | --- | --- | ---: |
| 1 | 0 | 肩部前后 | 向身体后方 | +1 |
| 2 | 1 | 肩部侧向 | 向身体内侧 | +1 |
| 3 | 2 | 上臂旋转 | 向身体外侧（逆时针） | +1 |
| 4 | 3 | 肘部前后/屈伸 | 向上；小臂从下方向面部移动 | +1 |
| 5 | 4 | 肘部/前臂旋转 | 顺时针 | +1 |
| 6 | 5 | 腕部前后（与 slot 6 同一腕部位置） | 向后 | +1 |
| 7 | 6 | 腕部左右（与 slot 5 同一腕部位置） | 向身体中心（顺时针） | +1 |
| 8 | 7 | 尚未定义 | 当前日志为 0.0 | 未定 |

### 5.3 右臂现场标定表

右臂 slot 0 的最终现场修正是“向前”，不是早期原始记录中的“向上”。

| SDK 关节编号 | SDK slot | 物理关节描述 | 穿戴者视角下的正方向 | 方向记录 |
| ---: | ---: | --- | --- | ---: |
| 1 | 0 | 肩部前后 | 向身体前方 | +1 |
| 2 | 1 | 肩部侧向 | 向身体外侧 | +1 |
| 3 | 2 | 上臂旋转 | 向身体内侧（逆时针） | +1 |
| 4 | 3 | 肘部前后/屈伸 | 向上；小臂从下方向面部移动 | +1 |
| 5 | 4 | 肘部/前臂旋转 | 顺时针、向身体外侧 | +1 |
| 6 | 5 | 腕部前后（与 slot 6 同一腕部位置） | 向后 | +1 |
| 7 | 6 | 腕部左右（与 slot 5 同一腕部位置） | 向身体外侧（顺时针） | +1 |
| 8 | 7 | 尚未定义 | 当前日志为 0.0 | 未定 |

这两张表描述的是**外骨骼源侧**的物理语义。表里的 `+1` 是现场记录，不表示任意屏幕坐标轴或下游设备一定使用相同正号。

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

实时模式默认是“相对显示”：每次重新连接后的第一帧作为显示基准，便于观察动作方向。它不是绝对的人体零位；查看协议绝对弧度时使用 `--absolute`。

### 6.4 单轴验证顺序

从自然下垂姿态开始，保持其他关节不动，每次只动一个 slot：

1. slot 0：确认肩部前后；
2. slot 1：确认肩部侧向；
3. slot 2：低头观察上臂绕轴旋转；
4. slot 3：确认大臂不动时小臂向面部/远离面部；
5. slot 4：低头观察前臂绕轴旋转；
6. slot 5、slot 6：确认两者在同一个腕部位置但绕不同方向转动。

如果原始监视器中预期 slot 没有变化，先检查协议、串口和插槽；如果原始值正确但 3D 方向相反，修正显示层符号；不要修改 `2π/16384`。

## 7. 外骨骼数据的使用规则

### 7.1 原始值、弧度和方向变换

外骨骼模块输出的 `left_arm_joint_rad`/`right_arm_joint_rad` 是协议层弧度。后续任何显示、记录或控制映射都应明确写出：

```text
source_rad[i] = signed_raw[i] * 2π / 16384
display_or_consumer_rad[j] = sign[j] * source_rad[source_slot[j]] + offset[j]
```

其中 `sign`、`source_slot` 和 `offset` 属于使用方的映射，不属于协议解码器。外骨骼自身的 slot 语义以第 5 节表格为准。

### 7.2 自然下垂基准

如果需要用动作增量，启动或重新连接后应在穿戴者保持自然下垂时记录：

```text
exo_neutral_rad[i] = 当前有效帧的 source_rad[i]
delta_exo[i] = source_rad[i] - exo_neutral_rad[i]
```

这样可以把“电机安装零位”和“穿戴者自然下垂姿态”分开。`--absolute` 只改变观察器是否显示相对基准，不会改变底层协议弧度，也不会替代正式标定。

### 7.3 不要融合独立电机

- slot 2 和 slot 4 都保留，并分别记录；
- slot 5 和 slot 6 虽然在同一个腕部位置，也分别保留；
- 不要在输入层求和、平均或用一个值覆盖另一个值；
- 如果下游只需要一个腕部自由度，应在下游映射中明确选择 slot 5 或 slot 6，并留下记录；
- slot 7 在定义之前保持未映射。

## 8. 开发和现场标定流程

推荐按以下顺序开发：

1. 不连接任何执行器，先通过 VID:PID 找到设备；
2. 用原始监视器确认 0～6 号 slot 的变化；
3. 用 3D 演示模式熟悉坐标、相机和机械链；
4. 接入实时 3D 观察器，按第 6.4 节逐轴确认方向；
5. 在自然下垂姿态记录 neutral raw/rad；
6. 记录每个 slot 的正方向、行程、中间点和极限；
7. 只修改一个符号、零位或比例，再重复单轴验证；
8. 最后验证 slot 2/4 的组合和 slot 5/6 的同腕部组合。

每个已验证的外骨骼通道至少保留以下记录：

| 臂 | slot | 自然下垂 raw | 自然下垂 rad | 单轴正向（穿戴者视角） | 显示符号 | 行程/备注 | 已验证 |
| --- | ---: | ---: | ---: | --- | ---: | --- | --- |
| 左/右 | 0～6 | 待测 | 待算 | 按第 5 节填写 | ±1 | 待测 | 否/是 |

出现方向异常时按这个顺序排查：

1. 串口设备是否确实是目标 VID:PID；
2. 是否把左臂和右臂 payload 偏移读反；
3. `int16` 符号和 little-endian 是否正确；
4. slot 编号是否把 SDK 1-based 和代码 0-based 混用；
5. 是否把穿戴者视角误当成观察者/屏幕视角；
6. 最后才修改显示或使用方的符号、零位和比例。

## 9. 测试说明

### 9.1 构建和运行外骨骼测试

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

### 9.2 `exoskeleton_runtime` 验证什么

[`tests/exoskeleton_runtime_test.cpp`](../tests/exoskeleton_runtime_test.cpp) 不需要真实外骨骼、USB 设备或执行器。它使用 POSIX pseudo-terminal 模拟串口：

1. 创建伪终端并取得从端路径；
2. 用显式伪终端路径启动 `Exoskeleton`，配置中仍带有 VID/PID 字段；
3. 从主端写入一帧 131 字节的合法 full 帧；
4. 左 slot 0 写入 `1234`，右 slot 0 写入 `-2345`；
5. 检查线程完成连接、原始值解析正确、弧度转换正确；
6. 关闭伪终端主端；
7. 检查最新状态被清空、连接失效、`stateFresh()` 为 false；
8. 停止线程并清理伪终端。

这个测试验证的是“串口线程和状态生命周期”，不验证人体方向和 3D 几何方向。方向验证必须通过真实动作配合第 5/6 节完成。

### 9.3 Python 工具的快速检查

```bash
python3 -m py_compile tools/exoskeleton_serial.py tools/exoskeleton_joint_monitor.py tools/exoskeleton_3d_viewer.py
python3 tools/exoskeleton_3d_viewer.py --demo --no-browser
```

演示服务启动后，可在浏览器中确认模型、绿色地面、鼠标旋转、WASD、升降和全屏按钮；关闭服务后不会触碰串口。

### 9.4 录制帧回放

为了稳定调整方向和零位，建议保存一组真实 full 帧作为离线输入。每次修改 slot、符号、比例或 offset 时都回放同一组数据，避免一边手动动作一边改变多个变量。

## 10. 安全边界和已知未定义项

- 同一时刻只允许一个程序打开外骨骼串口；监视器和实时观察器不能同时占用设备。
- 默认严格匹配 VID:PID；多个匹配设备时必须先消除歧义。
- 断开、读错误、重连和停止会清空最新状态；超过 stale 时间时至少会让 `stateFresh()` 失效，即使缓存对象还存在，上层也不得继续使用旧姿态。
- 输入层只读串口，不会向外骨骼发送电机控制命令。
- slot 7 尚未定义，当前日志虽常为 0.0，也不能据此推断它的物理含义。
- 3D 观察器用于验证外骨骼自身的空间方向，不是绝对姿态测量仪；相对模式的首帧是显示基准，不是人体医学零位。
- slot 2/4 和 slot 5/6 必须作为独立自由度保留；是否在其他系统中减少自由度，应由使用方明确选择。
- 修改方向表时，必须同时更新现场记录、显示层实现和测试说明，不能只改屏幕上的某一个负号。
