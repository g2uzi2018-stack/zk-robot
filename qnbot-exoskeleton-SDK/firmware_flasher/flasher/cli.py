from __future__ import annotations

import argparse
from pathlib import Path

from .command_utils import FlashToolError
from .dap_backend import PyOcdBackend
from .dfu_backend import DfuUtilBackend


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="firmware-flasher",
        description="Unified STM32 firmware flasher via dfu-util (DFU) and pyOCD (DAP).",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    list_p = sub.add_parser("list", help="List connected devices")
    list_p.add_argument("--backend", choices=["dfu", "dap"], required=True)

    flash_p = sub.add_parser("flash", help="Flash firmware")
    flash_p.add_argument("--backend", choices=["dfu", "dap"], required=True)
    flash_p.add_argument("--file", required=True, type=Path, help="Firmware file path")

    flash_p.add_argument("--address", help="Flash address for DFU .bin, e.g. 0x08000000")
    flash_p.add_argument("--usb-id", help="USB VID:PID filter for dfu-util, e.g. 0483:df11")
    flash_p.add_argument("--alt", type=int, help="DFU alt setting index")

    flash_p.add_argument("--probe", help="CMSIS-DAP unique id for pyOCD")
    flash_p.add_argument("--target", help="pyOCD target name, e.g. stm32f407vg")
    flash_p.add_argument("--frequency", type=int, help="SWD/JTAG frequency in Hz")

    return parser


def _get_backend(name: str):
    if name == "dfu":
        return DfuUtilBackend()
    return PyOcdBackend()


def run(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        backend = _get_backend(args.backend)

        if args.command == "list":
            print(backend.list_devices())
            return 0

        if args.command == "flash":
            output = backend.flash(
                args.file,
                address=args.address,
                usb_id=args.usb_id,
                alt=args.alt,
                probe=args.probe,
                target=args.target,
                frequency=args.frequency,
            )
            print(output)
            return 0

        parser.error(f"Unknown command: {args.command}")
        return 2
    except FlashToolError as exc:
        print(f"ERROR: {exc}")
        return 1

