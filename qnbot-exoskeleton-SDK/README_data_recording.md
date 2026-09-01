# 远程操作器数据记录功能说明

## 功能概述

远程操作器数据读取器现已支持完整的数据记录和分析功能，包括：

- 实时数据记录到CSV文件
- 自动生成关节位置和速度分析图表
- 摇杆数据可视化
- 详细的统计分析摘要

## 使用方法

### 1. 数据记录模式（推荐）

```bash
python remote_manipulator_data_reader.py --record
```

此模式专门用于数据记录，会：
- 显示实时记录状态（时长、数据点数、采样率）
- 按Ctrl+C结束后自动生成所有分析图表
- 生成详细的统计摘要

### 2. 控制台演示模式

```bash
python remote_manipulator_data_reader.py
```

基本的控制台演示，同时启用数据记录功能。

### 3. GUI演示模式

```bash
python remote_manipulator_data_reader.py --gui
```

图形界面演示，同时启用数据记录功能。

### 4. 帮助信息

```bash
python remote_manipulator_data_reader.py --help
```

## 生成的文件

程序运行结束后，会在同目录下生成以下文件：

### 1. CSV数据文件
- 文件名：`remote_manipulator_data_YYYYMMDD_HHMMSS.csv`
- 包含所有原始数据：时间戳、摇杆数据、关节原始值、关节弧度值

### 2. 图表目录
- 目录名：`charts_YYYYMMDD_HHMMSS/`
- 包含以下PNG图表：
  - `left_arm_joint_positions.png` - 左臂关节位置曲线
  - `right_arm_joint_positions.png` - 右臂关节位置曲线
  - `left_arm_joint_velocities.png` - 左臂关节速度曲线
  - `right_arm_joint_velocities.png` - 右臂关节速度曲线
  - `joystick_data.png` - 摇杆数据曲线

### 3. 分析摘要文件
- 文件名：`analysis_summary_YYYYMMDD_HHMMSS.txt`
- 包含：
  - 记录时长和数据点统计
  - 每个关节的统计信息（均值、标准差、最值）
  - 平均采样率

## 图表说明

### 关节位置图表
- 显示8个关节的位置变化（弧度制）
- 每个子图显示一个关节的时间序列
- 包含均值和标准差统计信息

### 关节速度图表
- 显示8个关节的速度变化（弧度/秒）
- 通过位置数据的数值微分计算
- 包含均值、标准差和最大速度统计信息

### 摇杆数据图表
- 显示左右摇杆的X、Y轴数据
- 实时反映操作者的控制输入

## 程序化使用

```python
from remote_manipulator_data_reader import RemoteManipulatorReader

# 创建读取器（启用数据记录）
reader = RemoteManipulatorReader(
    port="/dev/tty.usbmodemCMSIS_DAP2", 
    baudrate=2000000, 
    enable_logging=True
)

# 启动数据读取
reader.start()

# 你的应用逻辑...

# 停止读取（自动生成图表）
reader.stop()

# 可选：保存分析摘要
reader.save_analysis_summary()

# 获取记录的数据
logged_data = reader.get_logged_data()
```

## 依赖包

确保安装以下Python包：

```bash
pip install numpy matplotlib pyserial
```

对于GUI功能，还需要：

```bash
pip install PyQt5
```

## 注意事项

1. 数据记录功能默认启用，如不需要可设置 `enable_logging=False`
2. 图表生成需要matplotlib，请确保已安装
3. 速度计算采用简单的数值微分，可能包含噪声
4. 长时间记录会产生大量数据，注意磁盘空间
5. 程序结束时的图表生成可能需要几秒钟时间 
