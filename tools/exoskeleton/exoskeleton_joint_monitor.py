#!/usr/bin/env python3
"""Display legacy exoskeleton joint telemetry in a fixed terminal layout.

This monitor intentionally has no dependency on the vendor SDK. It reads the
legacy stream directly, using the same frame layout and encoder-to-radian
conversion as the official reference reader.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
import unicodedata
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from exoskeleton_serial import parse_usb_id, resolve_serial_port

try:
    import serial
except ImportError as error:  # pragma: no cover - environment dependent
    raise SystemExit("需要 pyserial：python3 -m pip install pyserial") from error


JOINT_COUNT = 8
VALUE_WIDTH = 34
FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55
PAYLOAD_LENGTHS = (128, 88, 48)
ENCODER_TO_RADIAN_RATIO = 2.0 * math.pi / 16384.0


@dataclass(frozen=True)
class TelemetrySnapshot:
    format_version: int
    left_arm_joint: list[int]
    right_arm_joint: list[int]

    @property
    def left_arm_joint_rad(self) -> list[float]:
        return [value * ENCODER_TO_RADIAN_RATIO for value in self.left_arm_joint]

    @property
    def right_arm_joint_rad(self) -> list[float]:
        return [value * ENCODER_TO_RADIAN_RATIO for value in self.right_arm_joint]


def parse_frame(frame: bytes) -> TelemetrySnapshot:
    """Parse one validated legacy frame."""
    payload_length = len(frame) - 3
    if payload_length not in (48, 88, 128):
        raise ValueError(f"不支持的 payload 长度: {payload_length}")
    if frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
        raise ValueError("legacy 帧头或帧尾错误")

    checksum = 0
    for value in frame[1:-2]:
        checksum ^= value
    if checksum != frame[-2]:
        raise ValueError("legacy 校验失败")

    payload = frame[1:-2]
    left = list(struct.unpack_from("<8h", payload, 16))
    right = list(struct.unpack_from("<8h", payload, 32))
    format_version = {48: 1, 88: 2, 128: 3}[payload_length]
    return TelemetrySnapshot(format_version, left, right)


class LegacySerialReader:
    """Small streaming reader for the official legacy telemetry format."""

    def __init__(self, port: str, baudrate: int) -> None:
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.02)
        self.buffer = bytearray()

    def close(self) -> None:
        self.serial.close()

    def read_snapshot(self) -> Optional[TelemetrySnapshot]:
        chunk = self.serial.read(self.serial.in_waiting or 1)
        if chunk:
            self.buffer.extend(chunk)
        return self._extract_snapshot()

    def _extract_snapshot(self) -> Optional[TelemetrySnapshot]:
        while True:
            try:
                head = self.buffer.index(FRAME_HEADER)
            except ValueError:
                self.buffer.clear()
                return None

            if head:
                del self.buffer[:head]
            if len(self.buffer) < 51:
                return None

            needs_more_data = False
            for payload_length in PAYLOAD_LENGTHS:
                frame_length = payload_length + 3
                if len(self.buffer) < frame_length:
                    needs_more_data = True
                    continue

                candidate = bytes(self.buffer[:frame_length])
                if candidate[-1] != FRAME_TAIL:
                    continue
                checksum = 0
                for value in candidate[1:-2]:
                    checksum ^= value
                if checksum != candidate[-2]:
                    continue

                del self.buffer[:frame_length]
                return parse_frame(candidate)

            if needs_more_data:
                return None
            del self.buffer[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="固定布局显示外骨骼左右臂 8 个编码器槽位"
    )
    parser.add_argument(
        "--port",
        default=None,
        help="显式指定串口路径；默认按 USB VID:PID 自动发现",
    )
    parser.add_argument(
        "--vid",
        type=parse_usb_id,
        default=0x0483,
        help="USB VID，默认 0x0483",
    )
    parser.add_argument(
        "--pid",
        type=parse_usb_id,
        default=0x5740,
        help="USB PID，默认 0x5740",
    )
    parser.add_argument("--baudrate", type=int, default=2_000_000)
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="终端刷新间隔（秒），默认 0.05",
    )
    return parser.parse_args()


def display_width(value: str) -> int:
    """Return the approximate terminal column width of a string."""
    width = 0
    for character in value:
        east_asian_width = unicodedata.east_asian_width(character)
        width += 2 if east_asian_width in ("W", "F") else 1
    return width


def value_text(raw: int, radians: float) -> str:
    return f"raw={raw:6d}  rad={radians:+10.4f}"


def make_layout() -> Tuple[list[str], Dict[Tuple[str, int], int], int, int]:
    """Create static labels and return value-line positions."""
    lines = [
        "外骨骼关节监视器（Ctrl-C 退出；仅读取 legacy 遥测）",
        "",
        "状态 : 等待遥测数据...",
        "",
        "左臂",
    ]
    positions: Dict[Tuple[str, int], int] = {}
    prefixes: list[str] = []

    for side, label in (("left", "左臂"), ("right", "右臂")):
        if side == "right":
            lines.extend(["", label])
        for index in range(JOINT_COUNT):
            prefix = f"  关节{index + 1} (slot {index}) : "
            prefixes.append(prefix)
            positions[(side, index)] = len(lines)
            lines.append(prefix + "等待数据")

    lines.extend(["", "说明：rad 是官方读取器换算的编码器弧度值；编号从 1 开始。"])
    value_column = display_width(prefixes[0]) + 1
    status_column = display_width("状态 : ") + 1
    return lines, positions, value_column, status_column


def update_layout(
    snapshot: TelemetrySnapshot,
    layout_rows: int,
    positions: Dict[Tuple[str, int], int],
    value_column: int,
    status_column: int,
) -> None:
    """Rewrite only value fields in the already printed terminal layout."""
    updates: Dict[int, str] = {2: f"format={snapshot.format_version}"}
    left_rad = snapshot.left_arm_joint_rad
    right_rad = snapshot.right_arm_joint_rad
    for index in range(JOINT_COUNT):
        updates[positions[("left", index)]] = value_text(
            snapshot.left_arm_joint[index], left_rad[index]
        )
        updates[positions[("right", index)]] = value_text(
            snapshot.right_arm_joint[index], right_rad[index]
        )

    output = [f"\033[{layout_rows}A"]
    for row in range(layout_rows):
        if row in updates:
            column = status_column if row == 2 else value_column
            output.append(f"\033[{column}G{updates[row]:<{VALUE_WIDTH}}")
        if row != layout_rows - 1:
            output.append("\033[1B")
    output.append("\033[1B")
    sys.stdout.write("".join(output))
    sys.stdout.flush()


def main() -> int:
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval 必须大于 0")

    lines, positions, value_column, status_column = make_layout()
    layout_rows = len(lines)
    sys.stdout.write("\033[?25l" + "\n".join(lines) + "\n")
    sys.stdout.flush()

    rendered_generation = -1
    latest: Optional[TelemetrySnapshot] = None
    generation = 0
    reader: Optional[LegacySerialReader] = None
    active_port: Optional[str] = None
    try:
        active_port = resolve_serial_port(args.port, args.vid, args.pid)
        reader = LegacySerialReader(active_port, args.baudrate)
        while True:
            snapshot = reader.read_snapshot()
            if snapshot is not None:
                latest = snapshot
                generation += 1
            if latest is not None and generation != rendered_generation:
                update_layout(
                    latest,
                    layout_rows,
                    positions,
                    value_column,
                    status_column,
                )
                rendered_generation = generation
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    except (serial.SerialException, RuntimeError) as error:
        binding = active_port or args.port or f"VID:PID={args.vid:04x}:{args.pid:04x}"
        print(f"\n无法打开或读取串口 {binding}: {error}", file=sys.stderr)
        return 1
    finally:
        if reader is not None:
            reader.close()
        sys.stdout.write("\033[?25h\033[0m\n")
        sys.stdout.flush()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
