# Qnbot Python SDK 使用说明

修订日期：2026-05-03

本文面向上位机开发和联调人员，说明如何使用 `qnbot_sdk_v1.2/` 目录下的 Qnbot Python SDK 与设备通信。SDK 同时支持 QnTP 请求响应、legacy 遥测流解析，以及 Wireless 配对与配置接口。

## 0. 机型支持矩阵

| 设备类型(device_type) | 设备名称 | QnTP 覆盖说明 |
|------|------|------|
| `0x0101` | `Qnbot Exo Std` | 覆盖 `System/DeviceProfile/Telemetry/Encoder/Handset/Imu/Haptics`，通常不包含 Wireless(`0x84`) |
| `0x0102` | `Qnbot Exo Plus` | 覆盖与 `0x0101` 类似，通常不包含 Wireless(`0x84`) |
| `0x0103` | `Qnbot Exo Plus Wireless / RF` | 在 `std/plus` 基础上额外支持 `Wireless(0x84)`，并支持 `System.Status(0x03)`、`System.EnterDfu(0x80)` |

机型 QnTP 差异说明：

- 三个机型均覆盖 IMU 输出总开关消息：`Imu.SetOutputEnable(0x20)` 和 `Imu.GetOutputEnable(0x21)`。
- `0x0101` 与 `0x0102` 的基础 QnTP 消息集合通常一致。
- `0x0103` 通常为超集，支持 Wireless 配对/配置链路和系统扩展消息。

## 1. 文件组成

| 文件 | 用途 |
|------|------|
| `qnbot_sdk.py` | Python SDK 主库，提供串口连接、协议组帧解析和高层 API。 |
| `qnbot_cli.py` | CLI 入口脚本。 |
| `cli_utils.py` | CLI 交互逻辑和输出格式。 |

推荐直接使用 `qnbot_sdk_v1.2/` 目录下文件进行联调。若需要发布独立 SDK 包，请同步复制 `qnbot_sdk.py`、`qnbot_cli.py`、`cli_utils.py` 和本文档。

## 2. 环境准备

要求：

- Python 3.8+
- 串口依赖 `pyserial`
- Windows 使用 `COMx` 端口，Linux/macOS 使用 `/dev/ttyACM0`、`/dev/tty.usbmodemXXXX` 等端口名

安装依赖：

```bash
pip install pyserial
```

在项目内运行 CLI 时：

```bash
cd qnbot_sdk_v1.2
python qnbot_cli.py --port COM5
```

Linux/macOS 示例：

```bash
cd qnbot_sdk_v1.2
python qnbot_cli.py --port /dev/ttyACM0
```

## 3. 快速开始

最小 Python 示例：

```python
from qnbot_sdk import QnbotClient

with QnbotClient(port="COM5", baudrate=2_000_000) as client:
    ver = client.get_system_version()
    st = client.get_system_status()
    print(f"type=0x{ver.device_type:04X} battery={st.battery_mv}mV/{st.battery_percent}%")
```

常用初始化参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | 必填 | 串口路径，例如 `COM5` 或 `/dev/ttyACM0`。 |
| `baudrate` | `2_000_000` | 串口波特率。USB CDC 场景通常不敏感。 |
| `protocol_mode` | `auto` | 解析模式：`auto`、`legacy`、`qntp`。 |
| `read_timeout` | `0.02` | 串口读取超时，单位秒。 |
| `dispatch_queue_size` | `2048` | 回调分发队列容量。 |

## 4. CLI 使用

启动交互式 CLI：

```bash
python qnbot_cli.py --port COM5
```

常用启动参数：

```bash
python qnbot_cli.py --port COM5 --debug
python qnbot_cli.py --port COM5 --telemetry-print
python qnbot_cli.py --port COM5 --protocol-mode qntp
python qnbot_cli.py --port COM5 --no-interactive --duration 10
python qnbot_cli.py --port COM5 --lang zh-CN
python qnbot_cli.py --port COM5 --lang en-US
```

语言资源说明：

- CLI 支持 `--lang auto|zh-CN|en-US`。
- `auto` 会按系统环境自动判定语言（无法判定时默认 `zh-CN`）。
- 语言资源位于 `locales/zh-CN.json` 与 `locales/en-US.json`。

进入 CLI 后输入 `help` 查看命令。当前支持：

| 命令 | 说明 |
|------|------|
| `sys.version` | 获取系统版本基础信息（不含状态扩展字段）。 |
| `sys.status` | 获取系统状态扩展信息（电池、无线链路、SN、设备名）。 |
| `sys.caps` | 获取能力位图。 |
| `sys.dfu` | 请求进入系统 DFU。 |
| `lang <auto\|zh-CN\|en-US>` | 在 CLI 运行中切换显示语言（Switch CLI language at runtime）。 |
| `telemetry on/off` | 开关遥测打印。 |
| `telemetry mode line/stream` | 切换遥测输出模式。 |
| `tm.stream <stream_id> <0\|1>` | 设置 Telemetry 运行时流开关，`stream_id` 支持 `0xF1`(LegacyProtocol) 和 `0x00`(HighRateSnapshot)。 |
| `dev.topo` | 获取设备拓扑信息。 |
| `dev.profiles` | 获取设备数据 profile，当前固件通常返回 `UNSUPPORTED`。 |
| `enc.list` | 获取编码器列表。 |
| `enc.info <ch>` | 获取单个编码器信息。 |
| `enc.zero.get <ch>` | 获取编码器零位。 |
| `enc.zero.set_here <ch>` | 将当前位置设为零位。 |
| `enc.zero.set <ch> <zero_value>` | 设置编码器零位值。 |
| `hand.list` | 获取手柄列表。 |
| `hand.info <idx>` | 获取单个手柄信息。 |
| `hand.calib.get <idx>` | 获取手柄校准参数。 |
| `hand.calib.start <idx>` | 开始手柄校准流程。 |
| `hand.calib.commit <idx> <point>` | 提交手柄校准点，`point` 支持 `1..8` 或 `jx_center` 等别名。 |
| `hand.calib.finish <idx>` | 结束手柄校准流程。 |
| `hand.calib.wizard [idx]` | 启动引导式手柄校准流程；若只检测到一个在线手柄，可省略 `idx`。 |
| `hand.out.get <idx>` | 读取手柄校准输出状态。 |
| `hand.out.set <idx> <0\|1>` | 设置手柄校准输出状态。 |
| `hand.param.set <idx> <point> <value>` | 设置手柄校准参数值。 |
| `imu.list` | 获取 IMU 列表。 |
| `imu.info <idx>` | 获取单个 IMU 信息。 |
| `imu.mag.start <idx>` | 开始 IMU 磁力计校准。 |
| `imu.mag.finish <idx>` | 结束 IMU 磁力计校准并保存。 |
| `imu.mag.wizard [idx]` | 启动引导式 IMU 磁力计校准。 |
| `imu.output.get` | 获取全局 IMU 运行时输出总开关状态。 |
| `imu.output.set <0\|1> [persist]` | 设置全局 IMU 输出总开关，`persist` 可选 `0/1`，默认 `1` 写入 Flash。 |
| `wl.status` | 获取 Wireless 状态。 |
| `wl.pair.start [role]` | 启动 Wireless 配对，`role` 可选 `master`、`slave`、`direct`。 |
| `wl.pair.wait [timeout_s]` | 等待配对流程进入终态。 |
| `wl.pair.run [role] [timeout_s]` | 启动配对并等待终态。 |
| `wl.pair.wizard [role] [timeout_s]` | 启动引导式 Wireless 配对（含预检查、状态轮询、step=6 等待与终态收尾）。 |
| `wl.pair.cancel` | 取消 Wireless 配对。 |
| `wl.reset` | 复位 Wireless 模块。 |
| `wl.config.enter` | 进入 Wireless 配置态。 |
| `wl.config.exit` | 退出 Wireless 配置态。 |
| `wl.stream <enable>` | 设置 Wireless 透传开关，`enable` 可选 `0` 或 `1`。 |
| `wl.push <freq>` | 设置 Wireless 推送频率档位，`freq` 可选 `2` 或 `4`。 |
| `wl.result <0..5> [peer_name]` | 写入配对结果和对端设备名。 |
| `haptics.out <ch> <amp> <pat> <ms>` | 设置 Haptics 输出；`ch` 为触觉通道，协议默认 `0=手柄主体`。 |
| `haptics.cal.get <idx>` | 获取 Haptics 驱动校准状态。 |
| `haptics.cal.run <idx>` | 触发 Haptics 驱动校准。 |
| `haptics.play <idx> <effect>` | 播放 Haptics 预置效果；常用效果可参考 `1/4/14/47/52/64`。 |
| `haptics.stop <idx>` | 停止 Haptics 输出。 |
| `haptics.rt <idx> <amp>` | 设置 Haptics 实时振幅，`amp` 常用范围 `0..127`。 |
| `haptics.enable <idx> <0\|1>` | 设置 Haptics 使能。 |
| `haptics.mode <idx> <mode>` | 设置 Haptics 模式；常用 `0=Linear`、`1=Threshold`、`2=Pulse`、`3=Curve`。 |
| `haptics.pressure <idx> <value>` | 设置 Haptics 压力值，范围通常为 `0..4095`。 |
| `haptics.timeout <idx> <ms>` | 设置 Haptics 超时值。 |
| `haptics.intensity <idx>` | 获取 Haptics 强度。 |
| `haptics.force.test <idx> <pressure> [mode] [timeout_ms] [hold_s]` | 力反馈定压测试：自动使能并周期下发压力，`hold_s=0` 时需按 `q` 停止。 |
| `haptics.trigger.sim <idx> [threshold] [poll_ms]` | 扳机震感模拟：按手柄扳机值实时映射压力（阈值触发），按 `q` 停止。 |
| `stats` | 查看解析统计。 |

配对 CLI 示例：

```text
wl.status
wl.pair.run slave 35
wl.status
```

`wl.pair.start` 的返回值只代表命令已受理，最终结果需要通过 `wl.pair.wait` 或 `wl.status` 判断。`role` 省略时沿用固件默认角色。

也可以直接用引导流程：

```text
wl.pair.wizard slave 30
```

说明：

- 流程会先读取 `wl.status` 做预检查：若已在配对/配置态，会先做取消或退出配置态。
- 轮询显示为固定三行刷新：轮询提示、当前 `Wireless 状态`、轮询状态文本。
- 在进入 `step=6` 前，轮询状态显示“准备中”；进入 `step=6` 后显示“正在等待配对结果”。
- 当配对进入终态时，会打印 `配对流程已进入终态：<结果>`（`OK/TIMEOUT/CANCELED/ERROR`）。
- 若在 `step=6` 后超过 `timeout_s` 仍未终态，会自动尝试 `wl.pair.cancel` 并回读终态结果。

### 4.1 CLI 命令参数说明

以下参数说明面向现场调试，帮助快速理解“每个位置该填什么”。

#### `lang <auto|zh-CN|en-US>`

| 参数 | 含义 | 建议 |
|------|------|------|
| `auto|zh-CN|en-US` | CLI 显示语言 | `auto` 自动跟随环境，现场建议显式指定 `zh-CN` 或 `en-US` |

#### `telemetry mode line|stream`

| 参数 | 含义 | 建议 |
|------|------|------|
| `line` | 固定区域刷新 | 适合日常观测，避免刷屏 |
| `stream` | 持续滚动输出 | 适合抓取长日志或配合重定向保存 |

#### `tm.stream <stream_id> <0|1>`

| 参数 | 含义 | 常用取值 |
|------|------|------|
| `stream_id` | Telemetry 运行时流 ID | `0xF1=LegacyProtocol`，`0x00=HighRateSnapshot` |
| `0|1` | 开关状态 | `1=开启`，`0=关闭` |

示例：

```text
tm.stream 0xF1 1
tm.stream 0xF1 0
```

#### `enc.info <ch>` / `enc.zero.get <ch>` / `enc.zero.set_here <ch>`

| 参数 | 含义 | 建议 |
|------|------|------|
| `ch` | 编码器通道号 `channel_index` | 常见范围 `0..15`；具体可先用 `enc.list` 查看 |

示例：

```text
enc.info 0
enc.zero.get 0
enc.zero.set_here 0
```

#### `enc.zero.set <ch> <zero_value>`

| 参数 | 含义 | 常用范围 / 建议 |
|------|------|------|
| `ch` | 编码器通道号 | 常见 `0..15` |
| `zero_value` | 零位值 | 建议 `0..16383`；通常为一圈原始计数范围内的值 |

#### `hand.info <idx>` / `hand.calib.get <idx>` / `hand.calib.start <idx>` / `hand.calib.finish <idx>` / `hand.out.get <idx>`

| 参数 | 含义 | 建议 |
|------|------|------|
| `idx` | 手柄索引 `handset_index` | 常见 `0=左`、`1=右`；先用 `hand.list` 确认 |

#### `hand.calib.commit <idx> <point>`

| 参数 | 含义 | 常用取值 |
|------|------|------|
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `point` | 校准点编号或别名 | 支持 `1..8`，也支持 `jx_center/jx_max/jx_min/jy_center/jy_max/jy_min/trig_start/trig_max` |

建议：

- 现场优先使用 `hand.calib.wizard [idx]`，避免手动逐点提交时顺序出错。

#### `hand.out.set <idx> <0|1>`

| 参数 | 含义 | 建议 |
|------|------|------|
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `0|1` | 校准输出开关 | `1=开启校准输出`，`0=关闭校准输出` |

#### `hand.param.set <idx> <point> <value>`

| 参数 | 含义 | 建议 |
|------|------|------|
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `point` | 校准参数点 | 与 `hand.calib.commit` 相同，支持 `1..8` 或别名 |
| `value` | 参数值 | 直接写入校准参数，通常来自向导采样结果 |

#### `imu.info <idx>` / `imu.mag.start <idx>` / `imu.mag.finish <idx>`

| 参数 | 含义 | 建议 |
|------|------|------|
| `idx` | IMU 索引 `imu_index` | 常见从 `0` 开始；先用 `imu.list` 确认 |

说明：

- `imu.mag.start <idx>` 开始磁力计校准。
- `imu.mag.finish <idx>` 结束校准并请求保存。

#### `imu.output.set <0|1> [persist]`

| 参数 | 含义 | 常用取值 |
|------|------|------|
| `0|1` | 全局 IMU 输出开关 | `1=开启`，`0=关闭` |
| `persist` | 是否持久化到 Flash | `1=写入 Flash`，`0=仅本次运行有效`；默认 `1` |

示例：

```text
imu.output.set 1
imu.output.set 0 0
```

#### `wl.pair.start [role]` / `wl.pair.run [role] [timeout_s]` / `wl.pair.wizard [role] [timeout_s]`

| 参数 | 含义 | 常用取值 |
|------|------|------|
| `role` | 配对角色模式 | `master`、`slave`、`direct` |
| `timeout_s` | 等待终态超时时间（秒） | 常用 `30` 或 `35` |

说明：

- `wl.pair.start` 仅发起，不等待结果。
- `wl.pair.run` 发起并等待终态。
- `wl.pair.wizard` 带预处理、轮询和终态提示，现场最省心。

#### `wl.pair.wait [timeout_s]`

| 参数 | 含义 | 建议 |
|------|------|------|
| `timeout_s` | 等待配对终态的超时秒数 | 常用 `30~35` |

#### `wl.stream <enable>`

| 参数 | 含义 | 常用取值 |
|------|------|------|
| `enable` | Wireless 透传开关 | `1=开启`，`0=关闭` |

#### `wl.push <freq>`

| 参数 | 含义 | 常用取值 |
|------|------|------|
| `freq` | 推送频率档位 | 当前固件通常仅支持 `2` 或 `4` |

#### `wl.result <0..5> [peer_name]`

| 参数 | 含义 | 常用取值 / 建议 |
|------|------|------|
| `0..5` | 配对结果码 | 常见 `0=NONE`、`1=RUNNING`、`2=OK`、`3=TIMEOUT`、`4=CANCELED`、`5=ERROR` |
| `peer_name` | 对端设备名 | 可选；通常在主模式配对成功后写入 |

示例：

```text
wl.result 2 3262334A3235_M
```

## 5. 常用 Python API

### 5.1 System

```python
with QnbotClient(port="COM5") as client:
    version = client.get_system_version()
    status = client.get_system_status()
    caps = client.get_system_capabilities()
```

`SystemVersionInfo` 主要字段：

| 字段 | 说明 |
|------|------|
| `status_code` | 状态码。 |
| `proto_major/proto_minor` | 协议版本。 |
| `device_type` | 设备类型。 |
| `platform/revision/feature/build` | 固件版本字段。 |

`SystemStatusInfo` 主要字段：

| 字段 | 说明 |
|------|------|
| `status_code` | 状态码。 |
| `battery_mv` | 电池电压，单位 mV。 |
| `battery_percent` | 电量百分比，`0xFF` 表示充电中或有线供电。 |
| `wireless_link_state` | Wireless 链路状态，`0/1`。 |
| `serial` | 设备 SN，原始 bytes。 |
| `device_name` | 设备名。 |

进入 DFU：

```python
status = client.enter_system_dfu()
print(f"DFU status=0x{status:02X}")
```

### 5.1.1 DeviceProfile 与 Telemetry 运行时流控制

```python
with QnbotClient(port="COM5") as client:
    topo = client.get_device_profile_topology()
    profiles = client.get_device_profile_data_profiles()
    client.set_telemetry_stream_runtime_config(0xF1, 1)  # 打开 legacy 遥测流
```

说明：

- `get_device_profile_topology()` 返回编码器、手柄、IMU 的有效数量和总槽位数。
- `get_device_profile_data_profiles()` 当前固件通常返回 `UNSUPPORTED(0x04)`，SDK 保留原始 payload 便于后续兼容。
- `set_telemetry_stream_runtime_config()` 当前常用 `stream_id=0xF1` 控制 legacy 遥测流开关；`stream_id=0x00` 对应 HighRateSnapshot，当前固件通常返回 `UNSUPPORTED`。

### 5.2 Encoder

```python
with QnbotClient(port="COM5") as client:
    enc_list = client.get_encoder_info_list()
    info = client.get_encoder_info(0)
    zero = client.get_encoder_zero_value(0)

    set_here = client.set_encoder_zero_here(0)
    set_value = client.set_encoder_zero_value(0, 8192)
```

说明：

- `channel_index` 为编码器通道号。
- `set_encoder_zero_here()` 当前主线固件返回 4 字节，历史固件可能额外返回 `angle_raw`，SDK 已做兼容。
- `set_encoder_zero_value()` 的 `zero_value` 范围建议保持在 `0~16383`。

### 5.2.1 Handset

```python
with QnbotClient(port="COM5") as client:
    hand_list = client.get_hand_info_list()
    hand0 = client.get_hand_info(0)
    calib = client.get_hand_calib_params(0)

    client.hand_calib_start(0)
    client.hand_calib_commit(0, 1)  # 1 = JX_CENTER
    client.hand_set_calib_output(0, 1)
    client.hand_set_calib_param(0, 7, 1200)  # 7 = TRIG_START
    client.hand_calib_finish(0)
```

校准点枚举：

| 值 | 名称 |
|----|------|
| `1` | `JX_CENTER` |
| `2` | `JX_MAX` |
| `3` | `JX_MIN` |
| `4` | `JY_CENTER` |
| `5` | `JY_MAX` |
| `6` | `JY_MIN` |
| `7` | `TRIG_START` |
| `8` | `TRIG_MAX` |

更适合现场人员的方式是直接使用 CLI 引导流程：

```text
hand.calib.wizard
```

说明：

- 若当前只检测到一个在线手柄，CLI 会自动选中该手柄。
- 向导采用固定 `8` 步 `SMART_CALIB_STEPS` 流程：
  1. 左滑摇杆后松开，采集左侧静止点
  2. 右滑摇杆后松开，采集右侧静止点
  3. 上滑摇杆后松开，采集上侧静止点
  4. 下滑摇杆后松开，采集下侧静止点
  5. 绕圈旋转摇杆约 6 秒，自动记录 X/Y 极值
  6. 松开扳机，采集扳机起始值
  7. 按下扳机到底约 3 秒，自动记录扳机最大值
  8. 写入校准参数并结束校准
- 向导开始前会自动执行 2 个准备动作：关闭校准输出（切原始值）、发送 `Hand.CalibStart`。
- 写入完成后会打印 `point=1..8` 的参数清单，并附带可读注释（如 `JX_CENTER(摇杆X中心)`），无需额外对照表。
- 向导期间使用固定刷新区域展示引导信息与左右手柄状态，避免流数据覆盖提示。

### 5.3 IMU

```python
with QnbotClient(port="COM5") as client:
    imu_list = client.get_imu_info_list()
    imu0 = client.get_imu_info(0)

    client.imu_mag_calibrate(0, 0x01)  # 开始校准
    # 缓慢绕 X/Y/Z 轴旋转设备
    client.imu_mag_calibrate(0, 0x02)  # 结束并保存

    client.set_imu_output_enable(0, persist=True)  # 关闭全部 IMU 输出并持久化
    state = client.get_imu_output_enable()         # 读取全局 IMU 输出总开关状态
```

`imu_mag_calibrate()` 的 `action`：

| 值 | 说明 |
|----|------|
| `0x01` | 开始磁力计校准。 |
| `0x02` | 结束磁力计校准并保存。 |

CLI 可直接使用引导模式：

```text
imu.mag.wizard
```

说明：

- 若设备只暴露一个可用 IMU，可省略 `idx`；若 `imu.list` 暂时为空，向导会回退尝试 `idx=0`。
- 向导会先发送 `start`，提示用户旋转采样，回车后自动发送 `finish` 并打印最终状态。
- 固件返回 `NOT_READY` 时会直接提示并结束，不会让用户停在半流程状态。

`set_imu_output_enable()` / `get_imu_output_enable()` 返回 `ImuOutputEnableResult`：

| 字段 | 说明 |
|------|------|
| `status_code` | 通用状态码，`0x00` 表示成功。 |
| `enable` | 当前 RAM 运行时全局总开关状态，`0=关闭`，`1=开启`。 |
| `persisted` | Flash 持久化状态或持久化是否完成。 |
| `capability_mask` | IMU 编译能力位图，`bit0=IMU0`，`bit1=IMU1`。 |
| `error_code` | `0=无错误`，`1=Flash 写入失败`，`2=固件无 IMU 能力`。 |

说明：

- `persist=True` 会请求固件写入 Flash，重启后保持全局 IMU 输出总开关。
- 关闭 IMU 是“上报级开关”，不会停止 UART/WIT 初始化或底层接收。
- 关闭后旧协议实时流 payload 缩短为 48 字节基础数据；打开后按编译能力恢复 IMU 数据长度；`imu.list` 不再列出 IMU。

## 6. Wireless 配对与配置

### 6.1 状态字段

```python
st = client.get_wireless_status_info()
print(st.pairing_busy, st.pair_step, st.pair_result, st.paired_device_name)
```

`WirelessStatusInfo` 主要字段：

| 字段 | 说明 |
|------|------|
| `status_code` | 状态码。 |
| `pairing_busy` | 配对流程是否忙，`0/1`。 |
| `pair_step` | 配对状态机阶段。 |
| `pair_result` | 配对结果。 |
| `in_config_mode` | 是否处于配置态。 |
| `link_state` | Wireless 链路引脚状态。 |
| `passthrough_enabled` | 透传开关。 |
| `push_freq_option` | 推送频率档位。 |
| `local_device_name` | 本机设备名。 |
| `paired_device_name` | 已配对设备名。 |

`pair_result` 枚举：

| 常量 | 值 | 说明 |
|------|----|------|
| `WIRELESS_PAIR_RESULT_NONE` | `0` | 无结果。 |
| `WIRELESS_PAIR_RESULT_RUNNING` | `1` | 配对进行中。 |
| `WIRELESS_PAIR_RESULT_OK` | `2` | 配对成功。 |
| `WIRELESS_PAIR_RESULT_TIMEOUT` | `3` | 配对超时。 |
| `WIRELESS_PAIR_RESULT_CANCELED` | `4` | 已取消。 |
| `WIRELESS_PAIR_RESULT_ERROR` | `5` | 配对错误。 |

`pair_step` 枚举：

| 常量 | 值 |
|------|----|
| `WIRELESS_PAIR_STEP_IDLE` | `0` |
| `WIRELESS_PAIR_STEP_PREPARE` | `1` |
| `WIRELESS_PAIR_STEP_SET_NAME` | `2` |
| `WIRELESS_PAIR_STEP_SET_ROLE` | `3` |
| `WIRELESS_PAIR_STEP_REPREPARE` | `4` |
| `WIRELESS_PAIR_STEP_SEND_CMD` | `5` |
| `WIRELESS_PAIR_STEP_WAIT_PEER` | `6` |
| `WIRELESS_PAIR_STEP_FINISHING` | `7` |

### 6.2 推荐配对流程

最省心的方式是直接启动并等待终态：

```python
from qnbot_sdk import (
    QnbotClient,
    WIRELESS_ROLE_SLAVE,
    WIRELESS_PAIR_RESULT_OK,
)

with QnbotClient(port="COM5") as client:
    result = client.wireless_pair_start_and_wait(
        role_mode=WIRELESS_ROLE_SLAVE,
        timeout=35.0,
        poll_interval=0.3,
    )

    final = result.final_status
    print(f"accepted=0x{result.accepted_status:02X}")
    print(f"elapsed={result.elapsed_s:.2f}s")
    print(f"pair_result={final.pair_result}, peer={final.paired_device_name}")

    if final.pair_result != WIRELESS_PAIR_RESULT_OK:
        raise RuntimeError("Wireless pairing did not finish successfully")
```

角色模式：

| 常量 | 值 | 说明 |
|------|----|------|
| `WIRELESS_ROLE_MASTER` | `0` | 主模式。 |
| `WIRELESS_ROLE_SLAVE` | `1` | 从模式。 |
| `WIRELESS_ROLE_DIRECT_PAIR` | `2` | 直接配对，跳过设置名称和角色步骤。 |

分步流程：

```python
from qnbot_sdk import QnbotClient, WIRELESS_ROLE_MASTER

with QnbotClient(port="COM5") as client:
    ack = client.wireless_pair_start(role_mode=WIRELESS_ROLE_MASTER)
    print(f"PairStart status=0x{ack.status_code:02X}")

    final = client.wait_wireless_pair_result(timeout=35.0, poll_interval=0.3)
    print(final.pair_result, final.paired_device_name)
```

注意：`wireless_pair_start()` 返回 `OK` 只表示固件已受理配对命令，不表示最终配对成功。最终结果以 `wait_wireless_pair_result()` 或 `get_wireless_status_info()` 的 `pair_result` 为准。

### 6.3 取消配对

```python
ack = client.wireless_pair_cancel()
print(f"PairCancel status=0x{ack.status_code:02X}")

final = client.wait_wireless_pair_result(timeout=5.0, poll_interval=0.2)
print(final.pair_result)
```

### 6.4 配置态和模块复位

```python
client.wireless_enter_config()
st = client.get_wireless_status_info()
print(st.in_config_mode)

client.wireless_exit_config()
client.wireless_reset()
```

配对进行中调用 `Reset/EnterConfig/ExitConfig`，固件可能返回 `STATUS_BUSY`。

### 6.5 透传和推送频率

```python
client.wireless_set_stream_config(1)  # 1=开启透传，0=关闭透传
client.wireless_set_push_freq(4)      # 仅支持 2 或 4
```

说明：

- `wireless_set_stream_config()` 仅允许 `0/1`。
- `wireless_set_push_freq()` 当前固件仅允许 `2/4`，对应约 `2ms/4ms`。
- 推送频率设置会写入 Flash，重启后仍应保持。

### 6.6 写入配对结果

```python
from qnbot_sdk import WIRELESS_PAIR_RESULT_OK

ack = client.wireless_set_pair_result_info(
    WIRELESS_PAIR_RESULT_OK,
    "PEER_DEVICE",
)
print(f"SetPairResultInfo status=0x{ack.status_code:02X}")

st = client.get_wireless_status_info()
print(st.pair_result, st.paired_device_name)
```

说明：

- SDK 默认按标准格式发送：`pair_result(1B) + reserved(3B) + paired_device_name(32B)`。
- `pair_result` 仅允许 `0~5`。
- `paired_device_name` 超过 32 字节会被截断，不足 32 字节会自动补零。
- 当 `pairing_busy=1` 时，固件会返回 `STATUS_BUSY`，避免覆盖正在进行的配对流程。

## 7. 遥测读取

使用回调读取实时遥测：

```python
from qnbot_sdk import QnbotClient, TelemetrySnapshot

def on_telemetry(snap: TelemetrySnapshot) -> None:
    print(snap.arm_joint_left_rad, snap.arm_joint_right_rad)

with QnbotClient(port="COM5") as client:
    client.register_telemetry_callback(on_telemetry)
    input("Press Enter to stop...")
```

`TelemetrySnapshot` 常用字段：

| 字段 | 说明 |
|------|------|
| `joystick_left/right` | 左右手柄原始 4 元组：`[axis_x, axis_y, key_mask, trigger]`。 |
| `arm_joint_left/right` | 左右臂编码器原始值。 |
| `arm_joint_left_rad/right_rad` | 左右臂关节弧度值。 |
| `torso_acc/gyro/quat` | 躯干 IMU 数据。 |
| `extra_acc/gyro/quat` | 附加 IMU 数据。 |
| `format_version` | legacy 数据格式版本。 |

手柄字段说明：

| 索引 | 字段 | 说明 |
|------|------|------|
| `0` | `axis_x` | 摇杆 X 轴原始值。 |
| `1` | `axis_y` | 摇杆 Y 轴原始值。 |
| `2` | `key_mask` | 按键/拨动开关位图。 |
| `3` | `trigger` | 扳机值。 |

`key_mask` 解析规则：

- 基础键 `B0~B4`：`bit0~bit4`，`0=按下`，`1=松开`（active-low）。
- 拨动开关：`bit5`，`1=ON`，`0=OFF`。
- `0x0102 / 0x0103` 机型扩展键：`bit8~bit12` 对应 `X0~X4`，同样采用 active-low。
- `bit6~bit7` 与 `bit13~bit15` 当前保留。

CLI 中会将上述字段显示为：

```text
左手柄: [raw=[1933, 2041, 7999, 4623]] X=1933,Y=2041 keys=0x1F3F(-) toggle=ON trigger=4623
```

## 8. Haptics

```python
with QnbotClient(port="COM5") as client:
    client.haptics_set_output(channel_id=0, amplitude=80, pattern_id=1, duration_ms=200)
    client.haptics_drv_get_cal_status(0)
    client.haptics_drv_calibrate(0)
    client.haptics_vibrate_play(0, 1)
    client.haptics_vibrate_realtime(0, 64)
    client.haptics_vibrate_stop(0)
    client.haptics_set_enable(0, 1)
    client.haptics_set_mode(0, 2)
    client.haptics_set_pressure(0, 1200)
    client.haptics_set_timeout(0, 500)
    client.haptics_get_intensity(0)
```

说明：

- 协议文档中 `Haptics` 当前标注为“编译关闭”，很多固件会返回 `UNSUPPORTED(0x04)`。
- SDK 已完整封装请求/响应，后续只要固件开放该能力即可直接联调，无需再改上位机接口。
- `Haptics.SetOutput` 的 `channel_id` 与 `handset_index` 不是一回事：`channel_id` 用于输出通道，协议默认 `0=手柄主体`；`handset_index` 则用于左/右手柄类命令，常见为 `0=左`、`1=右`。
- 若现场没有明确的通道映射信息，建议优先从 `channel_id=0` 开始验证，再根据设备实现调整。

命令参数说明：

### `haptics.out <ch> <amp> <pat> <ms>`

| 参数 | 含义 | 常用范围 / 建议 |
| --- | --- | --- |
| `ch` | 触觉输出通道 `channel_id` | 协议默认 `0=手柄主体`；若设备存在更多触点，则为 `1..N` |
| `amp` | 震动幅度 `amplitude` | 常用 `0..100`；建议先从 `60~80` 开始 |
| `pat` | 输出模式 `pattern_id` | 常见 `0=恒定`、`1=脉冲`、`2=呼吸`；实际以固件实现为准 |
| `ms` | 持续时间 `duration_ms` | 单位毫秒；`0` 通常表示持续输出，建议先用 `100~300` 验证 |

示例：

```text
haptics.out 0 80 0 200
```

含义：对默认触觉通道 `0` 输出一次 `80%` 左右强度、恒定模式、持续 `200ms` 的震动。

### `haptics.play <idx> <effect>`

| 参数 | 含义 | 常用范围 / 建议 |
| --- | --- | --- |
| `idx` | 手柄索引 `handset_index` | 常见 `0=左`、`1=右` |
| `effect` | 预置效果 ID | 固件常见范围 `1..123`；建议优先试 `1/4/14/47/52/64` |

示例：

```text
haptics.play 0 1
haptics.play 1 47
```

### `haptics.rt <idx> <amp>`

| 参数 | 含义 | 常用范围 / 建议 |
| --- | --- | --- |
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `amp` | 实时振幅 | 常用 `0..127`；可先试 `32/64/96` |

### `haptics.enable <idx> <0|1>`

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `0|1` | 是否使能力反馈 | `1=开启`，`0=关闭` |

常见用法：

```text
haptics.enable 0 1
haptics.enable 0 0
```

### `haptics.mode <idx> <mode>`

| 参数 | 含义 | 常用取值 |
| --- | --- | --- |
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `mode` | 力反馈模式 | `0=Linear`、`1=Threshold`、`2=Pulse`、`3=Curve` |

建议：

- 初次联调优先使用 `mode=0`。
- 需要做阈值触发感时再使用 `mode=1`。

### `haptics.pressure <idx> <value>`

| 参数 | 含义 | 常用范围 / 建议 |
| --- | --- | --- |
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `value` | 压力值 | 常用 `0..4095`；建议先从 `800~1200` 起步 |

### `haptics.timeout <idx> <ms>`

| 参数 | 含义 | 常用范围 / 建议 |
| --- | --- | --- |
| `idx` | 手柄索引 | 常见 `0=左`、`1=右` |
| `ms` | 超时时间（毫秒） | `0` 常表示不超时；建议调试时先用 `500` 或 `2000` |

### `haptics.intensity <idx>`

| 参数 | 含义 |
| --- | --- |
| `idx` | 手柄索引，常见 `0=左`、`1=右` |

说明：用于读取当前强度状态，适合确认实时振动或力反馈链路是否生效。

### `haptics.cal.get <idx>` / `haptics.cal.run <idx>`

| 参数 | 含义 |
| --- | --- |
| `idx` | 手柄索引，常见 `0=左`、`1=右` |

建议：

- `haptics.cal.get <idx>` 用于先检查当前马达校准状态。
- `haptics.cal.run <idx>` 用于触发自动校准，执行后建议再次 `haptics.cal.get <idx>` 确认结果。

推荐调试顺序：

1. `haptics.cal.get 0`：先确认驱动校准状态。
2. `haptics.play 0 1`：先用预置效果验证马达是否能正常触发。
3. `haptics.stop 0`：确认可正常停止。
4. `haptics.enable 0 1` + `haptics.mode 0 0` + `haptics.pressure 0 1200`：验证力反馈基础链路。
5. `haptics.timeout 0 2000`：设置超时，避免长时间保持输出。
6. `haptics.force.test 0 1200 0 2000 3`：做定压短时测试。
7. `haptics.trigger.sim 0 500 100`：做扳机联动测试。
8. `haptics.wizard`：现场优先使用，已整合通道探测、试播、固定反馈强度测试、扳机联动模拟。

常用预置效果示例：

| effect_id | 建议名称 | 用途说明 |
| --- | --- | --- |
| `1` | Strong Click | 强点击，适合先验证是否能触发。 |
| `4` | Sharp Click | 短促点击，适合按键反馈。 |
| `14` | Soft Bump | 柔和碰撞感，适合轻提示。 |
| `47` | Buzz | 蜂鸣型连续震动，便于确认持续输出。 |
| `52` | Pulsing | 脉冲型震动，适合节奏提示。 |
| `64` | Long Buzz | 长蜂鸣，适合持续提醒。 |

力反馈与扳机模拟建议：

- `haptics.mode <idx> <mode>` 常用模式：
  - `0=Linear`：压力值与输出强度线性对应，适合基础调试。
  - `1=Threshold`：适合做阈值触发类反馈。
  - `2=Pulse`：适合做脉冲式反馈。
  - `3=Curve`：适合后续做非线性反馈曲线。
- `haptics.pressure <idx> <value>` 常用范围为 `0..4095`，现场可先从 `800~1200` 起步。
- `haptics.wizard` 当前已内置两种更适合现场的流程：
  - 固定反馈强度测试：使用 `0..100` 的直观强度概念，便于快速判断“有没有感觉、强不强”。
  - 扳机联动模拟：等价于把 `haptics.trigger.sim` 并入向导，向导最后一步可直接进入实时联动模式。
- `haptics.trigger.sim <idx> [threshold] [poll_ms]` 中：
  - `threshold` 推荐先用 `500`。
  - 扳机值有效映射范围按业务规则为 `512..3584`。
  - 当 `threshold < 512` 时，SDK 仍会以 `512` 作为实际起振起点；超过该起点后再线性映射到 `pressure=0..4095`。
  - `poll_ms` 推荐 `50` 或 `100`；更小刷新更快，但串口请求会更密集。

## 9. 原始 QnTP 帧

需要调试底层协议时，可以注册 QnTP 回调：

```python
from qnbot_sdk import QnbotClient, QnTPFrame

def on_qntp(frame: QnTPFrame) -> None:
    print(frame.msg_type, frame.msg_class, frame.msg_id, frame.seq, frame.payload.hex())

with QnbotClient(port="COM5") as client:
    client.register_qntp_callback(on_qntp)
    client.get_system_version()
```

也可以直接组帧：

```python
from qnbot_sdk import build_qntp_frame, QNTP_MSG_REQUEST, MSGCLASS_SYSTEM, MSGID_SYSTEM_GET_VERSION

frame = build_qntp_frame(
    QNTP_MSG_REQUEST,
    MSGCLASS_SYSTEM,
    MSGID_SYSTEM_GET_VERSION,
    seq=0,
    payload=b"",
)
```

## 10. 状态码

| 常量 | 值 | 说明 |
|------|----|------|
| `STATUS_OK` | `0x00` | 成功。 |
| `STATUS_BAD_PARAM` | `0x01` | 参数非法。 |
| `STATUS_BUSY` | `0x02` | 设备忙。 |
| `STATUS_NOT_READY` | `0x03` | 子系统未就绪。 |
| `STATUS_UNSUPPORTED` | `0x04` | 功能不支持。 |

## 11. 联调检查清单

建议上位机联调按下面顺序验证：

1. `sys.version` 返回基础版本字段；`sys.status` 返回电池、SN、设备名和无线链路状态。
2. `sys.caps` 的能力位图包含 `System/Telemetry/Encoder/Handset/IMU`，`0x0103` 额外包含 `Wireless`。
3. `dev.topo` 能返回编码器、手柄、IMU 的数量与总槽位；`tm.stream 0xF1 1` 能正常返回。
4. `enc.list` 能返回编码器列表，`enc.zero.get` 能读到零位。
5. `hand.list` 能返回手柄列表，`hand.calib.get 0` 能读到校准参数。
6. `imu.list` 能返回 IMU 状态，`imu.mag.start/finish` 能正常响应。
7. `imu.output.get` 能返回全局 IMU 输出状态，`imu.output.set 0 0` 能临时关闭全部 IMU 输出。
8. `wl.status` 能返回本机名、配对结果和推送频率。
9. `wl.pair.run slave 35` 或 `wl.pair.run master 35` 能进入终态。
10. `wl.push 2` 和 `wl.push 4` 返回 `OK`，重启后 `wl.status` 能回读保持值。
11. `wl.result 2 PEER_DEVICE` 后，`wl.status` 能回读 `pair_result` 和 `paired_device_name`。
12. 若固件开放 `Haptics`，则 `haptics.intensity 0`、`haptics.enable 0 1` 等命令应能返回非 `UNSUPPORTED` 结果。

## 12. 常见问题

### PairStart 返回 OK，但没有配对成功

这是预期语义。`PairStart` 是受理型命令，`OK` 只表示固件开始执行配对流程。最终结果必须读取 `GetStatusInfo.pair_result`，或使用 `wireless_pair_start_and_wait()`。

### SetPushFreq 返回 BAD_PARAM

当前固件仅接受 `2` 或 `4`。旧文档中出现过 `1/2/4` 的描述，以当前 SDK 和固件为准。

### SetPairResultInfo 返回 BUSY

设备正在配对中，`pairing_busy=1`。请等待配对终态后再写入结果。

### 等待配对结果超时

默认建议总超时为 `35s`，覆盖固件约 `30s` 的配对窗口。若仍超时，优先检查 `wl.status` 中的 `pair_step`、`link_state` 和 `in_config_mode`。

### 串口响应超时

检查端口是否正确、是否被其他程序占用、设备是否上电、线缆是否稳定。调试时可加 `--debug` 查看原始 QnTP 收发帧。
