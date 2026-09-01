# Remote Manipulator Data Recording Guide

## Overview

The remote manipulator data reader supports full recording and analysis workflows, including:

- Real-time recording to CSV
- Automatic joint position/velocity chart generation
- Joystick data visualization
- Detailed statistics summary

## Usage

### 1. Recording Mode (Recommended)

```bash
python remote_manipulator_data_reader.py --record
```

This mode is dedicated to recording and will:

- Show live recording status (duration, sample count, sample rate)
- Auto-generate charts when you stop with Ctrl+C
- Export a detailed analysis summary

### 2. Console Demo Mode

```bash
python remote_manipulator_data_reader.py
```

Basic console demo with logging enabled.

### 3. GUI Demo Mode

```bash
python remote_manipulator_data_reader.py --gui
```

GUI demo with logging enabled.

### 4. Help

```bash
python remote_manipulator_data_reader.py --help
```

## Generated Files

After the program exits, the following files are generated in the working directory:

### 1. CSV Data File

- File name: `remote_manipulator_data_YYYYMMDD_HHMMSS.csv`
- Contains raw data including timestamps, joystick values, joint raw values, and joint radians

### 2. Charts Directory

- Directory name: `charts_YYYYMMDD_HHMMSS/`
- Contains PNG charts:
  - `left_arm_joint_positions.png`
  - `right_arm_joint_positions.png`
  - `left_arm_joint_velocities.png`
  - `right_arm_joint_velocities.png`
  - `joystick_data.png`

### 3. Analysis Summary File

- File name: `analysis_summary_YYYYMMDD_HHMMSS.txt`
- Includes:
  - Recording duration and sample statistics
  - Per-joint stats (mean, std, min, max)
  - Average sampling rate

## Chart Notes

### Joint Position Charts

- Show position trends (radians) for 8 joints
- Each subplot shows one joint over time
- Includes mean and standard deviation

### Joint Velocity Charts

- Show velocity trends (rad/s) for 8 joints
- Computed by numerical differentiation of position
- Includes mean, std, and peak velocity

### Joystick Chart

- Shows X/Y data for left/right joystick
- Reflects operator input over time

## Programmatic Usage

```python
from remote_manipulator_data_reader import RemoteManipulatorReader

reader = RemoteManipulatorReader(
    port="COM5",
    baudrate=2000000,
    enable_logging=True,
)

reader.start()

# Your application logic...

reader.stop()
reader.save_analysis_summary()

logged_data = reader.get_logged_data()
```

## Dependencies

Install required Python packages:

```bash
pip install numpy matplotlib pyserial
```

For GUI mode:

```bash
pip install PyQt5
```

## Notes

1. Logging is enabled by default. Disable with `enable_logging=False` if needed.
2. Chart generation requires `matplotlib`.
3. Velocity is computed numerically and may contain noise.
4. Long recordings may generate large files.
5. Chart rendering at exit may take a few seconds.
