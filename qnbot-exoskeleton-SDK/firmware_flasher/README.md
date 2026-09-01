# STM32 Firmware Flasher (DFU + DAP)

A cross-platform, Python-based firmware flashing toolkit using:

- `dfu-util` for USB DFU flashing
- `pyOCD` for CMSIS-DAP/SWD flashing

This project is designed as an MVP backend that can later be attached to a GUI (PySide6/PyQt).

## Features (MVP)

- Unified CLI for DFU and DAP workflows
- Firmware format validation (`.bin/.hex/.elf/.dfu`)
- Toolchain detection and clear error messages
- Device listing for both DFU and pyOCD targets

## Prerequisites

1. Python 3.9+
2. Install Python deps:

```bash
pip install -r firmware_flasher/requirements.txt
```

3. Install required dependencies:

- `dfu-util` (for DFU)
- `pyocd` command line (`pip install pyocd`) for DAP

## Quick Start

List connected DFU devices:

```bash
python firmware_flasher/main.py list --backend dfu
```

List connected CMSIS-DAP targets:

```bash
python firmware_flasher/main.py list --backend dap
```

Flash over DFU (`.bin` to address):

```bash
python firmware_flasher/main.py flash --backend dfu --file app.bin --address 0x08000000
```

Flash over DAP (`.elf/.hex/.bin`):

```bash
python firmware_flasher/main.py flash --backend dap --file app.elf
```

Optional: choose probe/target explicitly:

```bash
python firmware_flasher/main.py flash --backend dap --file app.hex --probe 123456789 --target stm32f407vg
```

## Firmware Type Guidance

- DFU path:
  - Preferred: `.bin` + explicit flash address
  - Also supported in many setups: `.dfu` (DfuSe container)
- DAP path:
  - `.elf` recommended (symbol + segment aware)
  - `.hex` and `.bin` supported

## Notes

- Not every STM32 supports USB DFU in system bootloader. Verify by part number against ST AN2606.
- This tool does not toggle BOOT0/NRST automatically yet; hardware or firmware-assisted boot mode switching is still needed.
