from __future__ import annotations

from pathlib import Path

from .command_utils import FlashToolError


DFU_SUFFIXES = {".bin", ".dfu", ".hex"}
DAP_SUFFIXES = {".bin", ".hex", ".elf"}


def ensure_file_exists(path: Path) -> None:
    if not path.exists() or not path.is_file():
        raise FlashToolError(f"Firmware file not found: {path}")


def validate_suffix(path: Path, allowed: set[str], backend_name: str) -> None:
    suffix = path.suffix.lower()
    if suffix not in allowed:
        allowed_text = ", ".join(sorted(allowed))
        raise FlashToolError(
            f"Unsupported firmware type for {backend_name}: {suffix or '<none>'}. "
            f"Allowed: {allowed_text}"
        )
