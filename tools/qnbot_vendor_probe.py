#!/usr/bin/env python3
"""Use the vendor Qnbot SDK to inspect an exoskeleton before robot control.

This helper deliberately imports the SDK supplied in
``qnbot-exoskeleton-SDK/qnbot_sdk_v1.2``.  It owns the serial port for the
duration of the probe; do not run it at the same time as a C++ monitor or
teleoperation process.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
VENDOR_SDK_ROOT = PROJECT_ROOT / "qnbot-exoskeleton-SDK" / "qnbot_sdk_v1.2"
sys.path.insert(0, str(VENDOR_SDK_ROOT))

from qnbot_sdk import (  # noqa: E402
    QnbotClient,
    TelemetrySnapshot,
    TELEMETRY_STREAM_ID_LEGACY_PROTOCOL,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="使用厂商 Qnbot SDK 检查外骨骼串口、版本和遥测流"
    )
    parser.add_argument("--port", default="/dev/exoskeleton", help="串口路径")
    parser.add_argument("--baudrate", type=int, default=2_000_000)
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="采集秒数；0 表示持续到 Ctrl-C",
    )
    parser.add_argument(
        "--enable-legacy-stream",
        action="store_true",
        help="通过厂商 QnTP API 请求打开 legacy 遥测流",
    )
    parser.add_argument(
        "--show-calibration",
        action="store_true",
        help="读取并打印 Hand.GetCalibParams，可直接填入遥操作 YAML",
    )
    parser.add_argument(
        "--show-topology",
        action="store_true",
        help="读取并打印厂商 DeviceProfile.GetTopology 和 Encoder.GetInfoList",
    )
    return parser.parse_args()


def format_snapshot(snapshot: TelemetrySnapshot) -> str:
    left = snapshot.joystick_left
    right = snapshot.joystick_right
    return (
        f"format={snapshot.format_version} "
        f"LJ={left} RJ={right} "
        f"EncL8={[round(value, 4) for value in snapshot.arm_joint_left_rad]} "
        f"EncR8={[round(value, 4) for value in snapshot.arm_joint_right_rad]}"
    )


def main() -> int:
    args = parse_args()
    latest: Optional[TelemetrySnapshot] = None
    latest_lock = threading.Lock()

    def on_telemetry(snapshot: TelemetrySnapshot) -> None:
        nonlocal latest
        with latest_lock:
            latest = snapshot

    try:
        with QnbotClient(
            port=args.port,
            baudrate=args.baudrate,
            protocol_mode="auto",
            read_timeout=0.02,
        ) as client:
            client.register_telemetry_callback(on_telemetry)

            try:
                version = client.get_system_version(timeout=2.0)
                print(
                    "System.GetVersion: "
                    f"status=0x{version.status_code:02X} "
                    f"device_type=0x{version.device_type:04X} "
                    f"proto={version.proto_major}.{version.proto_minor}"
                )
                caps = client.get_system_capabilities(timeout=2.0)
                print(
                    "System.GetCapabilities: "
                    f"status=0x{caps.status_code:02X} "
                    f"bitmap={caps.supported_class.hex()}"
                )
            except Exception as error:
                print(f"QnTP 查询失败（遥测仍会继续尝试）: {error}")

            if args.show_topology:
                try:
                    topology = client.get_device_profile_topology(timeout=2.0)
                    print(
                        "DeviceProfile.GetTopology: "
                        f"status=0x{topology.status_code:02X} "
                        f"encoders={topology.encoder_count}/"
                        f"{topology.encoder_total_slots} "
                        f"hands={topology.handset_count}/"
                        f"{topology.handset_total_slots} "
                        f"imus={topology.imu_count}/{topology.imu_total_slots}"
                    )
                    encoders = client.get_encoder_info_list(timeout=2.0)
                    print(
                        "Encoder.GetInfoList: "
                        f"status=0x{encoders.status_code:02X} "
                        f"count={len(encoders.entries)}"
                    )
                    for entry in encoders.entries:
                        print(
                            f"  slot={entry.channel_index} "
                            f"encoder_id=0x{entry.encoder_id:04X} "
                            f"hw=0x{entry.hw_version:04X} "
                            f"fw=0x{entry.fw_version:04X}"
                        )
                except Exception as error:
                    print(f"DeviceProfile/Encoder 查询失败: {error}")

            if args.show_calibration:
                try:
                    hand_list = client.get_hand_info_list(timeout=2.0)
                    print(
                        "Hand.GetInfoList: "
                        f"status=0x{hand_list.status_code:02X} "
                        f"count={len(hand_list.entries)}"
                    )
                    for entry in hand_list.entries:
                        calib = client.get_hand_calib_params(
                            entry.handset_index,
                            timeout=2.0,
                        )
                        side = "left" if entry.handset_index == 0 else "right"
                        print(
                            "Hand.GetCalibParams: "
                            f"status=0x{calib.status_code:02X} "
                            f"idx={calib.handset_index} "
                            f"output_enabled={calib.calib_output_enabled} "
                            f"jx=({calib.jx_center},{calib.jx_max},{calib.jx_min}) "
                            f"jy=({calib.jy_center},{calib.jy_max},{calib.jy_min}) "
                            f"trigger=({calib.trig_start},{calib.trig_max})"
                        )
                        if calib.status_code == 0x00:
                            print(
                                f"  {side}:\n"
                                f"    jx: {{center: {calib.jx_center}, "
                                f"max: {calib.jx_max}, min: {calib.jx_min}}}\n"
                                f"    jy: {{center: {calib.jy_center}, "
                                f"max: {calib.jy_max}, min: {calib.jy_min}}}\n"
                                f"    trigger: {{start: {calib.trig_start}, "
                                f"max: {calib.trig_max}}}\n"
                                "    deadzone_raw: 0"
                            )
                except Exception as error:
                    print(f"Hand.GetCalibParams 查询失败: {error}")

            if args.enable_legacy_stream:
                result = client.set_telemetry_stream_runtime_config(
                    TELEMETRY_STREAM_ID_LEGACY_PROTOCOL,
                    True,
                    timeout=2.0,
                )
                print(
                    "Telemetry.SetStreamRuntimeConfig: "
                    f"status=0x{result.status_code:02X} "
                    f"stream=0x{result.stream_id:02X}"
                )

            print(f"开始读取厂商 SDK 遥测: port={args.port}")
            deadline = None if args.duration <= 0 else time.monotonic() + args.duration
            last_print = 0.0
            printed_frames = 0
            while deadline is None or time.monotonic() < deadline:
                time.sleep(0.05)
                now = time.monotonic()
                if now - last_print < 0.2:
                    continue
                with latest_lock:
                    snapshot = latest
                if snapshot is None:
                    print("等待首个合法 legacy 遥测帧...")
                else:
                    print(format_snapshot(snapshot))
                    printed_frames += 1
                last_print = now

            stats = client.get_stats()
            print(f"SDK stats: {stats}")
            print(f"已打印遥测快照: {printed_frames}")
    except KeyboardInterrupt:
        print("收到 Ctrl-C，结束厂商 SDK 检查。")
    except Exception as error:
        print(f"厂商 SDK 检查失败: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
