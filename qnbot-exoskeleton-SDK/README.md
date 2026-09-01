# Qnbot Exoskeleton SDK

Python SDK and CLI for communicating with Qnbot exoskeleton devices over serial transport.

## Repository Layout

- `qnbot_sdk_v1.2/qnbot_sdk.py`: SDK core library
- `qnbot_sdk_v1.2/qnbot_cli.py`: interactive CLI entry
- `qnbot_sdk_v1.2/cli_utils.py`: CLI logic and telemetry rendering
- `qnbot_sdk_v1.2/SDK使用说明.md`: SDK usage guide (Chinese)
- `qnbot_sdk_v1.2/SDK_User_Guide.md`: SDK usage guide (English)
- `remote_manipulator_data_reader.py`: data logging and analysis utility
- `example.py`: minimal reader usage example
- `encoder_tool.py`: encoder debug utility
- `handler_tool.py`: handler calibration utility
- `qtp_package.py`: protocol frame/CRC helper

## Documentation Languages

- Chinese:
  - `README.md`
  - `README_data_recording.md`
  - `版本更新说明.md`
  - `qnbot_sdk_v1.2/SDK使用说明.md`
- English:
  - `README_en.md`
  - `README_data_recording_en.md`
  - `版本更新说明_en.md`
  - `qnbot_sdk_v1.2/SDK_User_Guide.md`

## Supported Device Types

- `0x0101`: Qnbot Exo Std
- `0x0102`: Qnbot Exo Plus
- `0x0103`: Qnbot Exo Plus Wireless / RF

Note:

- Feature availability is runtime-capability-driven (via `System.GetCapabilities`) and may differ by firmware build.
- The CLI auto-detects `device_type` and adjusts command visibility accordingly.

## Quick Start

1. Install dependency:

```bash
pip install pyserial
```

2. Run CLI:

```bash
cd qnbot_sdk_v1.2
python qnbot_cli.py --port COM5
```

3. Check device info:

```text
sys.version
sys.status
sys.caps
help
```

## License

Please add an open-source license file (for example `LICENSE`) before publishing.
