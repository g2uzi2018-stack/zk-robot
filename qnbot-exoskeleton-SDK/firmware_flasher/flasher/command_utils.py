from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass
from typing import Sequence


class FlashToolError(RuntimeError):
    """Raised when a flashing command fails or environment is invalid."""


@dataclass
class CommandResult:
    cmd: list[str]
    returncode: int
    stdout: str
    stderr: str


def check_tool_exists(tool_name: str) -> None:
    if shutil.which(tool_name) is None:
        raise FlashToolError(
            f"Required tool '{tool_name}' was not found in PATH. Install it and try again."
        )


def run_command(cmd: Sequence[str]) -> CommandResult:
    proc = subprocess.run(
        list(cmd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    result = CommandResult(
        cmd=list(cmd),
        returncode=proc.returncode,
        stdout=proc.stdout.strip(),
        stderr=proc.stderr.strip(),
    )
    if proc.returncode != 0:
        message = f"Command failed ({proc.returncode}): {' '.join(result.cmd)}"
        if result.stderr:
            message += f"\n{result.stderr}"
        raise FlashToolError(message)
    return result
