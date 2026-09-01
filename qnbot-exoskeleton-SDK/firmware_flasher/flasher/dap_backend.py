from __future__ import annotations

from pathlib import Path

from .backends import FlashBackend
from .command_utils import check_tool_exists, run_command
from .firmware import DAP_SUFFIXES, ensure_file_exists, validate_suffix


class PyOcdBackend(FlashBackend):
    def __init__(self) -> None:
        check_tool_exists("pyocd")

    def list_devices(self) -> str:
        result = run_command(["pyocd", "list"])
        return result.stdout or "No CMSIS-DAP probes found."

    def flash(self, firmware: Path, **kwargs: object) -> str:
        ensure_file_exists(firmware)
        validate_suffix(firmware, DAP_SUFFIXES, "DAP")

        probe = kwargs.get("probe")
        target = kwargs.get("target")
        frequency = kwargs.get("frequency")

        cmd = ["pyocd", "load", str(firmware)]
        if probe:
            cmd.extend(["-u", str(probe)])
        if target:
            cmd.extend(["-t", str(target)])
        if frequency:
            cmd.extend(["-f", str(frequency)])

        result = run_command(cmd)
        return result.stdout or "DAP flash completed."
