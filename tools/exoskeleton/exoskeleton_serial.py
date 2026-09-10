#!/usr/bin/env python3
"""Resolve the exoskeleton serial port by USB identity."""

from __future__ import annotations

from typing import Optional


def parse_usb_id(value: str) -> int:
    """Parse a USB id written as ``0x0483`` or ``0483``."""
    text = value.strip()
    if not text:
        raise ValueError("USB id must not be empty")
    if text.lower().startswith("0x"):
        text = text[2:]
    number = int(text, 16)
    if not 0 < number <= 0xFFFF:
        raise ValueError("USB id must be in the range 0x0001..0xFFFF")
    return number


def format_usb_id(vid: int, pid: int) -> str:
    return f"{vid:04x}:{pid:04x}"


def find_usb_serial_port(vid: int, pid: int) -> str:
    """Return the only serial port matching VID:PID, or raise clearly."""
    try:
        from serial.tools import list_ports
    except ImportError as error:  # pragma: no cover - handled by caller too
        raise RuntimeError(
            "按 VID:PID 自动发现需要 pyserial；请执行 "
            "python3 -m pip install pyserial"
        ) from error

    matches = sorted(
        str(info.device)
        for info in list_ports.comports()
        if info.vid == vid and info.pid == pid
    )
    identity = format_usb_id(vid, pid)
    if not matches:
        raise RuntimeError(f"没有找到 USB 串口 VID:PID={identity}")
    if len(matches) > 1:
        joined = ", ".join(matches)
        raise RuntimeError(
            f"USB 串口 VID:PID={identity} 匹配到多个设备：{joined}；"
            "请用 --port 显式指定"
        )
    return matches[0]


def resolve_serial_port(
    port: Optional[str],
    vid: int,
    pid: int,
) -> str:
    """Use an explicit path only when requested; otherwise discover by VID:PID."""
    if port:
        return port
    return find_usb_serial_port(vid, pid)
