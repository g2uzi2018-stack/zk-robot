from __future__ import annotations

from pathlib import Path

from .backends import FlashBackend
from .command_utils import FlashToolError, check_tool_exists, run_command
from .firmware import DFU_SUFFIXES, ensure_file_exists, validate_suffix


class DfuUtilBackend(FlashBackend):
    def __init__(self) -> None:
        check_tool_exists("dfu-util")

    def list_devices(self) -> str:
        result = run_command(["dfu-util", "-l"])
        return result.stdout or "No DFU devices found."

    def flash(self, firmware: Path, **kwargs: object) -> str:
        ensure_file_exists(firmware)
        validate_suffix(firmware, DFU_SUFFIXES, "DFU")

        address = kwargs.get("address")
        usb_id = kwargs.get("usb_id")
        alt = kwargs.get("alt")

        cmd = ["dfu-util"]
        if usb_id:
            cmd.extend(["-d", str(usb_id)])
        if alt is not None:
            cmd.extend(["-a", str(alt)])

        suffix = firmware.suffix.lower()
        if suffix == ".bin":
            if not address:
                raise FlashToolError("DFU .bin flashing requires --address, e.g. 0x08000000")
            cmd.extend(["-s", f"{address}:leave"])
        elif suffix == ".dfu":
            # DfuSe container usually includes target metadata.
            pass
        elif suffix == ".hex":
            if address:
                cmd.extend(["-s", f"{address}:leave"])

        cmd.extend(["-D", str(firmware)])
        result = run_command(cmd)
        return result.stdout or "DFU flash completed."
