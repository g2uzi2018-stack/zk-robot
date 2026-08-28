# TI5 T170C Controller 层设计

## 1. 分层位置

TI5 保持与 TIAGo 相同的理解方式：

```text
本体 CAN -> Motor -> Joint -> Arm / Head -> ArmController / HeadController
手部 CAN -> HandTransport -> Hand -> HandController
各部件 Controller -> Motion / Executor
```

灵巧手不使用本体电机协议，所以不经过 Motor 和 Joint；但 `Hand` 与 Arm、
Head 一样属于机器人部件层。

Controller 只负责保存最新目标和执行一个周期，不创建线程，不决定周期频率，
也不生成轨迹。上层 Motion 将已经规划好的每周期位置点交给 Controller，
实际发送统一发生在 `update()`。

## 2. 可以沿用 TIAGo 的部分

- `setTarget()` 只替换最新目标，不直接访问 CAN；
- 多个目标在一个控制周期内到达时，以最后一个为准；
- `update()` 是唯一的周期下发入口；
- Controller 不拥有 Arm、Head 或 Hand，它们的生命周期由外部管理；
- Controller 不负责插值、逆运动学、路径规划、线程和消息队列；
- 非法新目标先被完整拒绝，原有目标不变；
- 控制状态采用 `Idle / Running / Failed`。

## 3. TI5 不能照搬 TIAGo 的部分

### 3.1 Arm 和 Head 没有速度参数

TIAGo 的位置命令同时带有速度限制。TI5 当前确认的是 `0x44` 位置命令，
没有确认可随每个位置点下发的独立速度字段。因此 TI5 Controller 的
`setTarget()` 只接收位置；速度和加速度约束必须在上层生成每周期位置点时
落实，不能在 Controller 中虚构一个不会传到驱动器的参数。

### 3.2 开始控制必须从实测当前位置接管

Arm 或 Head 先由部件层完成：

1. `prepare()` 读取位置、驱动器范围、运行模式和故障；
2. `startPositionControlAtCurrentPosition()` 以当前位置建立并验证位置控制；
3. Controller 的 `start()` 再读取新鲜反馈，并把实测位置保存为初始目标。

Controller 的 `start()` 本身不发送位置命令，因此第一次 `update()` 不会从
调用方提供的旧位置突然跳到另一个位置。

自然下垂超出驱动器目标范围时，仍先使用专用恢复工具处理；Controller 不
复制肩部边界接管逻辑。

### 3.3 Arm 和 Head 的停止必须查询确认

`stopAndConfirm()` 调用部件层的 `0x02` STOP，并确认全部关节进入
`mode=0、fault=0`。STOP 不代表去使能、释放转矩或抱闸，现场仍必须可靠
承托机械臂。

Controller 运行失败后不会继续周期访问硬件，但仍允许显式调用
`stopAndConfirm()`，避免错误状态封死已经确认的停止通道。

### 3.4 TI5 Hand 是六通道灵巧手

HandController 保存六路原始位置和六路速度字节，并由 `update()` 下发
已经确认的 `0x50` 命令。只有配置中的 `protocol_verified` 和
`control_enabled` 同时为真才允许启动控制。

当前文档没有给出可信的灵巧手停止命令，所以 Controller 不提供假停止。
`pause()` 只停止后续 `0x50` 刷新，不表示灵巧手已停止、释放或断力。

## 4. 周期调用约束

- 同一个 Arm、Head 或 Hand 及其 Controller 只能由一个控制循环访问；
- 调用方负责按固定周期调用 `update()`；
- Arm 和 Head 的目标必须是相应周期的已规划位置点；
- `Failed` 后先记录错误并判断机械状态，再执行已确认的恢复或停止流程；
- `reset()` 只复位 Controller 的逻辑状态，不替代部件层和硬件恢复。

## 5. 当前范围

本层包含左臂、右臂共用的 ArmController、三关节 HeadController，以及左右
灵巧手共用的 HandController。腰部尚未纳入，等腰部协议和部件层确认后
再按同样层级接入。
