# Qnbot Python SDK User Guide

Revision date: 2026-05-29

This guide is intended for host-side developers and integration engineers. It describes how to use the Python SDK under `qnbot_sdk_v1.2/` to communicate with Qnbot devices. The SDK supports QnTP request/response, legacy telemetry parsing, and Wireless pairing/configuration interfaces.

## 0. Device Support Matrix

| Device Type (`device_type`) | Device Name | QnTP Coverage |
|------|------|------|
| `0x0101` | `Qnbot Exo Std` | `System/DeviceProfile/Telemetry/Encoder/Handset/Imu/Haptics`, usually without `Wireless(0x84)` |
| `0x0102` | `Qnbot Exo Plus` | Similar to `0x0101`, usually without `Wireless(0x84)` |
| `0x0103` | `Qnbot Exo Plus Wireless / RF` | Superset of std/plus, adds `Wireless(0x84)`, `System.Status(0x03)`, `System.EnterDfu(0x80)` |

Model differences:

- All three models support IMU global output switch messages: `Imu.SetOutputEnable(0x20)` and `Imu.GetOutputEnable(0x21)`.
- `0x0101` and `0x0102` usually share the same base QnTP message set.
- `0x0103` is usually a superset with Wireless pairing/configuration capabilities and extra system messages.

## 1. File Layout

| File | Purpose |
|------|------|
| `qnbot_sdk.py` | Core Python SDK library (serial transport, framing/parsing, high-level APIs). |
| `qnbot_cli.py` | CLI entry script. |
| `cli_utils.py` | CLI interaction logic and output formatting. |
| `SDK使用说明.md` | Chinese SDK usage guide. |
| `SDK_User_Guide.md` | English SDK usage guide. |

## 2. Environment Setup

Requirements:

- Python 3.8+
- `pyserial`
- Serial port naming: `COMx` on Windows, `/dev/ttyACM0` or `/dev/tty.usbmodemXXXX` on Linux/macOS

Install dependency:

```bash
pip install pyserial
```

## 3. Quick Start

Minimal Python example:

```python
from qnbot_sdk import QnbotClient

with QnbotClient(port="COM5", baudrate=2_000_000) as client:
    ver = client.get_system_version()
    st = client.get_system_status()
    print(f"type=0x{ver.device_type:04X} battery={st.battery_mv}mV/{st.battery_percent}%")
```

Common initialization parameters:

| Parameter | Default | Description |
|------|--------|------|
| `port` | required | Serial path, for example `COM5` or `/dev/ttyACM0`. |
| `baudrate` | `2_000_000` | Serial baudrate. Usually not sensitive for USB CDC transport. |
| `protocol_mode` | `auto` | Parser mode: `auto`, `legacy`, `qntp`. |
| `read_timeout` | `0.02` | Serial read timeout in seconds. |
| `dispatch_queue_size` | `2048` | Callback dispatch queue capacity. |

## 4. CLI Usage

Run interactive CLI:

```bash
cd qnbot_sdk_v1.2
python qnbot_cli.py --port COM5
```

Common startup options:

```bash
python qnbot_cli.py --port COM5 --debug
python qnbot_cli.py --port COM5 --telemetry-print
python qnbot_cli.py --port COM5 --protocol-mode qntp
python qnbot_cli.py --port COM5 --no-interactive --duration 10
python qnbot_cli.py --port COM5 --lang zh-CN
python qnbot_cli.py --port COM5 --lang en-US
```

Language resources:

- CLI supports `--lang auto|zh-CN|en-US`.
- `auto` chooses language by system locale and falls back to `zh-CN` if detection fails.
- Locale resources are under `locales/zh-CN.json` and `locales/en-US.json`.

After entering CLI, run `help` to list commands. Main commands include:

| Command | Description |
|------|------|
| `sys.version` | Get base system version info. |
| `sys.status` | Get extended system status (battery, wireless link, SN, device name). |
| `sys.caps` | Get capability bitmap. |
| `sys.dfu` | Request system DFU mode. |
| `lang <auto\|zh-CN\|en-US>` | Switch CLI language at runtime. |
| `telemetry on/off` | Enable or disable telemetry display. |
| `telemetry mode line/stream` | Set telemetry output mode. |
| `tm.stream <stream_id> <0\|1>` | Set telemetry runtime stream switch. |
| `dev.topo` | Get device topology. |
| `dev.profiles` | Get data profiles (often `UNSUPPORTED` on current firmware). |
| `enc.*` | Encoder commands: list/info/zero operations. |
| `hand.*` | Handset list/info/calibration/output/parameter commands. |
| `imu.*` | IMU list/info/mag calibration/output switch commands. |
| `wl.*` | Wireless status/pairing/configuration commands. |
| `haptics.*` | Haptics output/calibration/control commands (firmware-dependent). |
| `haptics.out <ch> <amp> <pat> <ms>` | Haptics output by channel. Protocol default is usually `channel 0 = main handset body`. |
| `haptics.play <idx> <effect>` | Play a preset haptics effect. Common effect IDs include `1/4/14/47/52/64`. |
| `haptics.force.test <idx> <pressure> [mode] [timeout_ms] [hold_s]` | Fixed-pressure force-feedback test. Enables haptics and sends pressure periodically. Press `q` to stop when `hold_s=0`. |
| `haptics.trigger.sim <idx> [threshold] [poll_ms]` | Trigger-to-pressure haptic simulation. Maps trigger value to pressure above threshold. Press `q` to stop. |
| `stats` | Show parser statistics. |

### 4.1 CLI Parameter Reference

The table below is intended for on-site debugging so you can quickly tell what each placeholder means.

#### `lang <auto|zh-CN|en-US>`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `auto|zh-CN|en-US` | CLI display language | Use `auto` for environment-based selection, or explicitly set `zh-CN` / `en-US` on site |

#### `telemetry mode line|stream`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `line` | Fixed refresh area | Best for daily observation, avoids scroll spam |
| `stream` | Continuous scrolling output | Best for long log capture |

#### `tm.stream <stream_id> <0|1>`

| Parameter | Meaning | Common values |
|------|------|------|
| `stream_id` | Telemetry runtime stream ID | `0xF1=LegacyProtocol`, `0x00=HighRateSnapshot` |
| `0|1` | Switch state | `1=enable`, `0=disable` |

Examples:

```text
tm.stream 0xF1 1
tm.stream 0xF1 0
```

#### `enc.info <ch>` / `enc.zero.get <ch>` / `enc.zero.set_here <ch>`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `ch` | Encoder `channel_index` | Commonly `0..15`; check `enc.list` first |

#### `enc.zero.set <ch> <zero_value>`

| Parameter | Meaning | Typical range / recommendation |
|------|------|------|
| `ch` | Encoder channel | Commonly `0..15` |
| `zero_value` | Zero offset value | Recommended `0..16383` |

#### `hand.info <idx>` / `hand.calib.get <idx>` / `hand.calib.start <idx>` / `hand.calib.finish <idx>` / `hand.out.get <idx>`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `idx` | `handset_index` | Commonly `0=left`, `1=right`; check `hand.list` first |

#### `hand.calib.commit <idx> <point>`

| Parameter | Meaning | Common values |
|------|------|------|
| `idx` | Handset index | Commonly `0=left`, `1=right` |
| `point` | Calibration point number or alias | `1..8`, or aliases such as `jx_center`, `jy_max`, `trig_start` |

Recommendation:

- Prefer `hand.calib.wizard [idx]` for on-site calibration to avoid wrong manual step ordering.

#### `hand.out.set <idx> <0|1>`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `idx` | Handset index | Commonly `0=left`, `1=right` |
| `0|1` | Calibration output switch | `1=enable`, `0=disable` |

#### `hand.param.set <idx> <point> <value>`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `idx` | Handset index | Commonly `0=left`, `1=right` |
| `point` | Calibration parameter point | Same mapping as `hand.calib.commit` |
| `value` | Parameter value | Usually written from guided calibration results |

#### `imu.info <idx>` / `imu.mag.start <idx>` / `imu.mag.finish <idx>`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `idx` | `imu_index` | Usually starts from `0`; check `imu.list` first |

#### `imu.output.set <0|1> [persist]`

| Parameter | Meaning | Common values |
|------|------|------|
| `0|1` | Global IMU output switch | `1=enable`, `0=disable` |
| `persist` | Persist to Flash | `1=write to Flash`, `0=RAM only`; default `1` |

Examples:

```text
imu.output.set 1
imu.output.set 0 0
```

#### `wl.pair.start [role]` / `wl.pair.run [role] [timeout_s]` / `wl.pair.wizard [role] [timeout_s]`

| Parameter | Meaning | Common values |
|------|------|------|
| `role` | Pairing role mode | `master`, `slave`, `direct` |
| `timeout_s` | Timeout in seconds while waiting for terminal result | Commonly `30` or `35` |

Notes:

- `wl.pair.start` only starts the flow.
- `wl.pair.run` starts and waits for terminal state.
- `wl.pair.wizard` is the most convenient on-site flow.

#### `wl.pair.wait [timeout_s]`

| Parameter | Meaning | Recommendation |
|------|------|------|
| `timeout_s` | Wait timeout in seconds | Commonly `30..35` |

#### `wl.stream <enable>`

| Parameter | Meaning | Common values |
|------|------|------|
| `enable` | Wireless passthrough switch | `1=enable`, `0=disable` |

#### `wl.push <freq>`

| Parameter | Meaning | Common values |
|------|------|------|
| `freq` | Push frequency option | Current firmware typically accepts only `2` or `4` |

#### `wl.result <0..5> [peer_name]`

| Parameter | Meaning | Common values / recommendation |
|------|------|------|
| `0..5` | Pair result code | Typically `0=NONE`, `1=RUNNING`, `2=OK`, `3=TIMEOUT`, `4=CANCELED`, `5=ERROR` |
| `peer_name` | Peer device name | Optional; usually written after successful master-mode pairing |

## 5. Key Python APIs

### 5.1 System

```python
with QnbotClient(port="COM5") as client:
    version = client.get_system_version()
    status = client.get_system_status()
    caps = client.get_system_capabilities()
```

`SystemVersionInfo` key fields:

- `status_code`
- `proto_major/proto_minor`
- `device_type`
- `platform/revision/feature/build`

`SystemStatusInfo` key fields:

- `status_code`
- `battery_mv`
- `battery_percent`
- `wireless_link_state`
- `serial`
- `device_name`

### 5.2 DeviceProfile and Telemetry Runtime Streams

```python
with QnbotClient(port="COM5") as client:
    topo = client.get_device_profile_topology()
    profiles = client.get_device_profile_data_profiles()
    client.set_telemetry_stream_runtime_config(0xF1, 1)
```

Notes:

- `get_device_profile_topology()` returns valid counts and total slots for encoders, handsets, and IMUs.
- `get_device_profile_data_profiles()` is often `UNSUPPORTED(0x04)` on current firmware.
- `set_telemetry_stream_runtime_config()` commonly uses `stream_id=0xF1` for legacy telemetry stream.

### 5.3 Encoder

```python
with QnbotClient(port="COM5") as client:
    enc_list = client.get_encoder_info_list()
    info = client.get_encoder_info(0)
    zero = client.get_encoder_zero_value(0)
    client.set_encoder_zero_here(0)
    client.set_encoder_zero_value(0, 8192)
```

### 5.4 Handset Calibration

```python
with QnbotClient(port="COM5") as client:
    hand_list = client.get_handset_info_list()
    calib = client.get_handset_calib_parameters(0)
```

Recommended operator flow:

```text
hand.calib.wizard [idx]
```

Wizard behavior:

- Uses fixed-step guided calibration.
- Uses fixed-line refresh so prompts are not flooded by telemetry.
- Prints final written calibration points with readable labels.

### 5.5 IMU

```python
with QnbotClient(port="COM5") as client:
    imu_list = client.get_imu_info_list()
    client.imu_mag_calibrate(0, 0x01)  # start
    # rotate device slowly around X/Y/Z axes
    client.imu_mag_calibrate(0, 0x02)  # finish and save

    client.set_imu_output_enable(0, persist=True)
    state = client.get_imu_output_enable()
```

### 5.6 Wireless

```python
with QnbotClient(port="COM5") as client:
    s = client.get_wireless_status_info()
    client.wireless_pair_start(role=1)             # slave
    final = client.wait_wireless_pair_result(35.0)
```

CLI shortcuts:

```text
wl.pair.run slave 35
wl.pair.wizard slave 30
```

Important notes:

- `wl.pair.start` returning `OK` only means request accepted, not final success.
- Check final result through `wl.pair.wait` or `wl.status`.
- For current firmware, `wl.push` typically accepts only `2` or `4`.

## 6. Telemetry Data Notes

`TelemetrySnapshot` common fields:

- `joystick_left/right`: `[axis_x, axis_y, key_mask, trigger]`
- `arm_joint_left/right`
- `arm_joint_left_rad/right_rad`
- `torso_acc/gyro/quat`
- `extra_acc/gyro/quat`
- `format_version`

`key_mask` decoding:

- Base keys `B0~B4`: `bit0~bit4`, active-low (`0=pressed`, `1=released`)
- Toggle switch: `bit5`, `1=ON`, `0=OFF`
- Extended keys on `0x0102/0x0103`: `bit8~bit12` as `X0~X4`, also active-low

## 7. Status Codes

| Constant | Value | Meaning |
|------|------|------|
| `STATUS_OK` | `0x00` | Success |
| `STATUS_BAD_PARAM` | `0x01` | Invalid parameter |
| `STATUS_BUSY` | `0x02` | Device busy |
| `STATUS_NOT_READY` | `0x03` | Subsystem not ready |
| `STATUS_UNSUPPORTED` | `0x04` | Not supported |

## 8. Validation Checklist

Recommended integration checks:

1. `sys.version`, `sys.status`, `sys.caps` are all readable.
2. `dev.topo` and `tm.stream 0xF1 1` return expected statuses.
3. Encoder list/read/zero commands are functional.
4. Handset list/calibration read and wizard complete normally.
5. IMU list/mag calibration/output switch are functional.
6. Wireless status/pairing/push settings are functional (for `0x0103`).
7. Haptics commands return meaningful status (`UNSUPPORTED` is acceptable if firmware closes that feature).

## 10. Haptics Notes

- `channel_id` and `handset_index` are different concepts:
  - `channel_id` is used by `Haptics.SetOutput`; protocol default is usually `0 = main handset body`.
  - `handset_index` is used by left/right handset commands, commonly `0 = left`, `1 = right`.
- If the field mapping is unknown on-site, start with `channel_id=0` for output verification.

Parameter reference:

### `haptics.out <ch> <amp> <pat> <ms>`

| Parameter | Meaning | Typical range / recommendation |
| --- | --- | --- |
| `ch` | Output `channel_id` | Protocol default is usually `0 = main handset body`; extra touch points may be `1..N` |
| `amp` | Output amplitude | Usually `0..100`; start with `60..80` |
| `pat` | Output pattern `pattern_id` | Commonly `0=constant`, `1=pulse`, `2=breathing`; firmware-specific |
| `ms` | Duration in milliseconds | `0` usually means continuous; start with `100..300` |

Example:

```text
haptics.out 0 80 0 200
```

### `haptics.play <idx> <effect>`

| Parameter | Meaning | Typical range / recommendation |
| --- | --- | --- |
| `idx` | `handset_index` | Commonly `0 = left`, `1 = right` |
| `effect` | Preset effect ID | Common firmware range `1..123`; try `1/4/14/47/52/64` first |

Examples:

```text
haptics.play 0 1
haptics.play 1 47
```

### `haptics.rt <idx> <amp>`

| Parameter | Meaning | Typical range / recommendation |
| --- | --- | --- |
| `idx` | `handset_index` | Commonly `0 = left`, `1 = right` |
| `amp` | Realtime amplitude | Commonly `0..127`; try `32/64/96` |

### `haptics.enable <idx> <0|1>`

| Parameter | Meaning | Recommendation |
| --- | --- | --- |
| `idx` | `handset_index` | Commonly `0 = left`, `1 = right` |
| `0|1` | Force-feedback enable | `1=enable`, `0=disable` |

### `haptics.mode <idx> <mode>`

| Parameter | Meaning | Typical values |
| --- | --- | --- |
| `idx` | `handset_index` | Commonly `0 = left`, `1 = right` |
| `mode` | Force-feedback mode | `0=Linear`, `1=Threshold`, `2=Pulse`, `3=Curve` |

Recommendation:

- Start with `mode=0` for initial bring-up.
- Use `mode=1` when testing threshold-triggered feedback.

### `haptics.pressure <idx> <value>`

| Parameter | Meaning | Typical range / recommendation |
| --- | --- | --- |
| `idx` | `handset_index` | Commonly `0 = left`, `1 = right` |
| `value` | Pressure value | Usually `0..4095`; practical starting point `800..1200` |

### `haptics.timeout <idx> <ms>`

| Parameter | Meaning | Typical range / recommendation |
| --- | --- | --- |
| `idx` | `handset_index` | Commonly `0 = left`, `1 = right` |
| `ms` | Timeout in milliseconds | `0` often means no timeout; start with `500` or `2000` |

### `haptics.intensity <idx>`

| Parameter | Meaning |
| --- | --- |
| `idx` | `handset_index`, commonly `0 = left`, `1 = right` |

Used to read current intensity state and verify whether realtime vibration or force-feedback is active.

### `haptics.cal.get <idx>` / `haptics.cal.run <idx>`

| Parameter | Meaning |
| --- | --- |
| `idx` | `handset_index`, commonly `0 = left`, `1 = right` |

Recommendation:

- Use `haptics.cal.get <idx>` to check current motor calibration state first.
- Use `haptics.cal.run <idx>` to trigger motor auto-calibration, then query again with `haptics.cal.get <idx>`.

Recommended validation flow:

1. `haptics.cal.get 0`
2. `haptics.play 0 1`
3. `haptics.stop 0`
4. `haptics.enable 0 1`
5. `haptics.mode 0 0`
6. `haptics.pressure 0 1200`
7. `haptics.timeout 0 2000`
8. `haptics.force.test 0 1200 0 2000 3`
9. `haptics.trigger.sim 0 500 100`
10. `haptics.wizard` for the preferred on-site combined flow: probing, trial playback, fixed feedback strength test, and trigger-linked simulation.

Common preset effects:

| effect_id | Name | Suggested use |
| --- | --- | --- |
| `1` | Strong Click | First smoke-test effect |
| `4` | Sharp Click | Button-like short feedback |
| `14` | Soft Bump | Gentle notification |
| `47` | Buzz | Continuous buzz for output confirmation |
| `52` | Pulsing | Rhythmic pulse feedback |
| `64` | Long Buzz | Sustained alert |

Force-feedback tuning references:

- Common modes:
  - `0=Linear`
  - `1=Threshold`
  - `2=Pulse`
  - `3=Curve`
- Typical pressure range: `0..4095`
- Practical starting point: `800..1200`
- `haptics.wizard` now includes two on-site-friendly flows:
  - fixed feedback strength test using an intuitive `0..100` strength scale
  - trigger-linked simulation integrated directly into the wizard
- For `haptics.trigger.sim`, a good initial setup is:
  - `threshold=500`
  - trigger effective range follows the business rule `512..3584`
  - if `threshold < 512`, the SDK still uses `512` as the actual feedback start point before mapping to `pressure=0..4095`
  - `poll_ms=50` or `100`

## 9. FAQ

### `PairStart` returns OK but pairing did not succeed

This is expected. `PairStart` is an acceptance-type command. Use `wl.pair.wait` or `wl.status` for final pairing result.

### `wl.push` returns BAD_PARAM

Current firmware usually accepts only `2` or `4`.

### Serial response timeout

Check serial port selection, exclusive port usage, power state, and cable stability. Use `--debug` for raw QnTP frame diagnostics.
