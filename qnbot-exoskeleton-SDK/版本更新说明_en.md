# Qnbot Exoskeleton SDK Release Notes

Updated on: 2026-06-04  
Repository: `qnbot-exoskeleton-SDK`

## 1. CLI Runtime Localization

- Added CLI language options:
  - `--lang auto`
  - `--lang zh-CN`
  - `--lang en-US`
- Added runtime language switch command:
  - `lang <auto|zh-CN|en-US>`

Notes:

- `auto` selects language based on system locale.
- If detection fails, CLI falls back to `zh-CN`.

## 2. New Locale Resource Directory

- Added: `qnbot_sdk_v1.2/locales/`
- Current resource files:
  - `zh-CN.json`
  - `en-US.json`

Notes:

- CLI output text is mapped through locale replacement resources for future language expansion.

## 3. Documentation Sync

- `qnbot_sdk_v1.2/SDK使用说明.md` has been updated with language-related usage:
  - startup option `--lang auto|zh-CN|en-US`
  - runtime command `lang <auto|zh-CN|en-US>`
  - `locales` directory notes

## 4. Compatibility and Impact

- Core protocol call behavior remains backward compatible.
- CLI updates are additive enhancements and preserve existing usage patterns.
- Recommended validation: run basic connectivity checks on both Windows and Linux (serial connect, command execution, factory flow).

## 5. Haptics Wizard and Debugging Experience Enhancements

- `haptics.*` commands are now always visible in CLI `help`, making it easier to confirm available capabilities across different device variants on site.
- `haptics.wizard` has been expanded into a more complete on-site debugging flow, including:
  - automatic probing of available channels and handsets
  - combined `SetOutput + VibratePlay` trial playback
  - fixed feedback strength test
  - trigger-linked simulation
- Probe output in `haptics.wizard` has been trimmed so `NOT_READY(0x03)` items are no longer shown; only meaningful results are displayed.

Notes:

- When the firmware returns `OK` for available channels/handsets, the wizard will prioritize those combinations automatically.
- If the field channel mapping is already known, the recommended values can still be overridden manually.

## 6. Fixed Feedback Strength Test Reworked

- The previous step-based pressure loop inside `haptics.wizard`, which focused on `pressure=0..4095`, has been replaced by a more intuitive fixed feedback strength test:
  - uses `fixed_strength=0..100`
  - maps it internally to a pressure value and sends it continuously for about `3` seconds
- This behavior now better matches the fixed feedback test used by the upper-computer control panel and avoids confusion with the earlier short trial playback duration.

Notes:

- The wizard now prints:
  - fixed feedback strength `fixed_strength`
  - mapped internal pressure value
- If a timeout occurs during the test, the timeout count will be recorded in the field summary for later serial-link or firmware diagnosis.

## 7. Trigger-Linked Simulation Integrated into the Wizard

- The standalone command remains available:
  - `haptics.trigger.sim <idx> [threshold] [poll_ms]`
- The same capability has also been integrated into the final step of `haptics.wizard`, so the entire on-site flow can be completed in one guided run.

Notes:

- The current trigger-linked logic follows the business rule effective range `512..3584`.
- When `threshold < 512`, the actual feedback start point still remains `512`.
- Runtime display includes:
  - current trigger value
  - user threshold
  - effective start point
  - current mapped pressure value

## 8. Documentation Sync

- `qnbot_sdk_v1.2/SDK使用说明.md` has been updated with:
  - Haptics parameter reference
  - current `haptics.wizard` flow description
  - fixed feedback strength test and trigger-linked simulation notes
- `qnbot_sdk_v1.2/SDK_User_Guide.md` has been updated accordingly in English.
- `qnbot_sdk_v1.2/locales/en-US.json` has been updated with the new Haptics wizard prompt strings.
