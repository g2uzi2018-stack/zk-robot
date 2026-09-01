#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
远程操作器数据读取模块
通过串口读取远程操作器的数据，并提供接口给上层应用使用

主要功能：
- 实时读取远程操作器数据（摇杆和关节数据）
- 自动记录数据到CSV文件
- 程序结束时自动生成分析图表
- 支持关节位置和速度曲线绘制
- 提供数据统计分析摘要

使用示例：

1. 基本使用方法（带数据记录）：
```python
# 导入模块
from remote_manipulator_data_reader import RemoteManipulatorReader

# 创建读取器实例（默认启用数据记录）
reader = RemoteManipulatorReader(port="/dev/tty.usbmodem326A335732351", baudrate=2000000)

# 方法1：使用回调函数
def on_data_received(data):
    print(f"收到新数据:\n{data}")
    # 处理数据...
    
# 注册回调函数
reader.register_callback(on_data_received)

# 启动数据读取
reader.start()

# 方法2：主动获取数据
# 启动数据读取
reader.start()

# 读取数据
latest_data = reader.get_latest_data()
print(f"主动获取的最新数据:\n{latest_data}")

# 使用完成后停止读取（会自动生成图表）
reader.stop()

# 可选：保存分析摘要
reader.save_analysis_summary()
```

2. 禁用数据记录：
```python
# 如果不需要数据记录功能，可以禁用
reader = RemoteManipulatorReader(port="/dev/tty.usbmodem326A335732351", baudrate=2000000, enable_logging=False)
```

2. 在ROS2中使用示例：
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import Joy

# 导入远程操作器数据读取模块
from remote_manipulator_data_reader import RemoteManipulatorReader

class RemoteManipulatorNode(Node):
    def __init__(self):
        super().__init__('remote_manipulator_node')
        
        # 创建发布者
        self.joy_left_pub = self.create_publisher(Joy, 'joy_left', 10)
        self.joy_right_pub = self.create_publisher(Joy, 'joy_right', 10)
        self.arm_joint_left_pub = self.create_publisher(
            Float32MultiArray, 'arm_joint_left', 10)
        self.arm_joint_right_pub = self.create_publisher(
            Float32MultiArray, 'arm_joint_right', 10)
        
        # 创建定时器，10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 创建读取器实例
        self.reader = RemoteManipulatorReader(
            port="/dev/tty.usbmodem326A335732351", 
            baudrate=2000000
        )
        
        # 启动数据读取
        if not self.reader.start():
            self.get_logger().error('无法启动数据读取器')
            rclpy.shutdown()
            return
        
        self.get_logger().info('远程操作器节点已启动')
    
    def timer_callback(self):
        # 获取最新数据
        data = self.reader.get_latest_data()
        
        # 发布左摇杆数据
        joy_left_msg = Joy()
        joy_left_msg.header.stamp = self.get_clock().now().to_msg()
        joy_left_msg.axes = [data.joystick_left[0]/512.0 - 1.0, data.joystick_left[1]/512.0 - 1.0]  # 归一化到[-1,1]
        joy_left_msg.buttons = [1 if data.joystick_left[2] > 0 else 0]
        self.joy_left_pub.publish(joy_left_msg)
        
        # 发布右摇杆数据
        joy_right_msg = Joy()
        joy_right_msg.header.stamp = self.get_clock().now().to_msg()
        joy_right_msg.axes = [data.joystick_right[0]/512.0 - 1.0, data.joystick_right[1]/512.0 - 1.0]  # 归一化到[-1,1]
        joy_right_msg.buttons = [1 if data.joystick_right[2] > 0 else 0]
        self.joy_right_pub.publish(joy_right_msg)
        
        # 发布左臂关节数据 (使用弧度制)
        arm_left_msg = Float32MultiArray()
        arm_left_msg.layout.dim = [MultiArrayDimension(label="joints", size=len(data.arm_joint_left_rad), stride=1)]
        arm_left_msg.data = [float(val) for val in data.arm_joint_left_rad]
        self.arm_joint_left_pub.publish(arm_left_msg)
        
        # 发布右臂关节数据 (使用弧度制)
        arm_right_msg = Float32MultiArray()
        arm_right_msg.layout.dim = [MultiArrayDimension(label="joints", size=len(data.arm_joint_right_rad), stride=1)]
        arm_right_msg.data = [float(val) for val in data.arm_joint_right_rad]
        self.arm_joint_right_pub.publish(arm_right_msg)
    
    def destroy_node(self):
        # 停止数据读取
        if hasattr(self, 'reader'):
            self.reader.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RemoteManipulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. 使用回调方式在ROS2中处理数据：
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import threading

# 导入远程操作器数据读取模块
from remote_manipulator_data_reader import RemoteManipulatorReader, RemoteManipulatorData

class RemoteManipulatorCallbackNode(Node):
    def __init__(self):
        super().__init__('remote_manipulator_callback_node')
        
        # 创建发布者
        self.data_pub = self.create_publisher(
            Float32MultiArray, 'manipulator_data', 10)
        
        # 创建读取器实例
        self.reader = RemoteManipulatorReader(
            port="/dev/tty.usbmodem326A335732351", 
            baudrate=2000000
        )
        
        # 注册回调函数
        self.reader.register_callback(self.data_callback)
        
        # 启动数据读取
        if not self.reader.start():
            self.get_logger().error('无法启动数据读取器')
            rclpy.shutdown()
            return
            
        self.get_logger().info('远程操作器回调节点已启动')
    
    def data_callback(self, data: RemoteManipulatorData):
        # 在回调函数中处理数据并发布
        # 注意：此回调函数在非ROS线程中执行，需要特别注意线程安全
        
        # 将所有数据合并为一个Float32MultiArray消息
        msg = Float32MultiArray()
        msg.layout.dim = [MultiArrayDimension(label="data", size=40, stride=1)]  # 更新数据大小：摇杆8 + 关节原始16 + 关节弧度16 = 40
        
        # 合并所有数据 (使用弧度制关节数据)
        all_data = []
        all_data.extend([float(val) for val in data.joystick_left])
        all_data.extend([float(val) for val in data.joystick_right])
        all_data.extend([float(val) for val in data.arm_joint_left_rad])
        all_data.extend([float(val) for val in data.arm_joint_right_rad])
        
        msg.data = all_data
        
        # 发布消息
        # 注意：由于这是在非ROS线程中调用，需要确保线程安全
        self.data_pub.publish(msg)
        
        # 记录日志
        # 注意：避免频繁记录日志，会影响性能
        # self.get_logger().info('收到新数据')
    
    def destroy_node(self):
        # 停止数据读取
        if hasattr(self, 'reader'):
            self.reader.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RemoteManipulatorCallbackNode()
    
    try:
        # 运行ROS2节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

import serial
import struct
import threading
import time
import queue
import logging
import math
import csv
import os
from datetime import datetime
from typing import Callable, Optional, List, Dict, Any

try:
    import numpy as np
except ImportError:
    np = None

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("RemoteManipulatorReader")


def _require_analysis_dependencies() -> None:
    if np is None or plt is None:
        raise ImportError("缺少分析依赖，请安装 numpy 和 matplotlib")

# 常量定义
REPORT_FRAME_HEADER = 0xAA
REPORT_FRAME_TAIL = 0x55

# 支持的载荷长度（不含帧头/校验/帧尾）
# 旧版基础: 摇杆(8*int16) + 关节(16*int16) = 48B
# 新版(四元数): 基础48B + (fAcc[3]+fGyro[3]+fQuat[4]) = 48 + (10*4) = 88B
# 新版(四元数+附加IMU): 48 + 40(躯干) + 40(附加) = 128B
DATA_LEN_BASE = 48
DATA_LEN_TORSO_IMU_QUAT = 88
DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT = 128
SUPPORTED_DATA_LENGTHS = [DATA_LEN_BASE, DATA_LEN_TORSO_IMU_QUAT, DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT]

# 编码器转换常量：16384对应2π弧度
ENCODER_TO_RADIAN_RATIO = 2 * math.pi / 16384

# 远程操作器数据结构
class RemoteManipulatorData:
    def __init__(self):
        self.joystick_left = [0, 0, 0, 0]  # X轴, Y轴, 按键事件, 板机
        self.joystick_right = [0, 0, 0, 0]  # X轴, Y轴, 按键事件, 板机
        self.arm_joint_left = [0] * 8  # 原始编码器数据
        self.arm_joint_right = [0] * 8  # 原始编码器数据
        self.arm_joint_left_rad = [0.0] * 8  # 转换为弧度制的数据
        self.arm_joint_right_rad = [0.0] * 8  # 转换为弧度制的数据
        self.timestamp = 0  # 时间戳，记录数据接收时间
        # 躯干IMU（如果报文包含）
        self.torso_acc = [0.0, 0.0, 0.0]
        self.torso_gyro = [0.0, 0.0, 0.0]
        # 兼容旧字段（欧拉角），保持占位
        self.torso_angle = [0.0, 0.0, 0.0]
        # 新增：四元数
        self.torso_quat = [0.0, 0.0, 0.0, 0.0]  # q0, q1, q2, q3
        # 附加IMU（头部等，如报文包含）
        self.extra_acc = [0.0, 0.0, 0.0]
        self.extra_gyro = [0.0, 0.0, 0.0]
        # 兼容旧字段（欧拉角），保持占位
        self.extra_angle = [0.0, 0.0, 0.0]
        # 新增：四元数
        self.extra_quat = [0.0, 0.0, 0.0, 0.0]  # q0, q1, q2, q3
        # 实际解析到的格式版本: 1(旧), 2(躯干IMU), 3(躯干+附加IMU)
        self.format_version = 1

    def to_dict(self) -> Dict[str, Any]:
        """将数据转换为字典格式"""
        return {
            "timestamp": self.timestamp,
            "joystick_left": self.joystick_left,
            "joystick_right": self.joystick_right,
            "arm_joint_left": self.arm_joint_left,
            "arm_joint_right": self.arm_joint_right,
            "arm_joint_left_rad": self.arm_joint_left_rad,
            "arm_joint_right_rad": self.arm_joint_right_rad,
            "torso_acc": self.torso_acc,
            "torso_gyro": self.torso_gyro,
            "torso_angle": self.torso_angle,
            "torso_quat": self.torso_quat,
            "extra_acc": self.extra_acc,
            "extra_gyro": self.extra_gyro,
            "extra_angle": self.extra_angle,
            "extra_quat": self.extra_quat,
            "format_version": self.format_version,
        }

    def __str__(self) -> str:
        """友好的字符串表示"""
        return (f"时间戳: {self.timestamp}\n"
                f"左摇杆: X={self.joystick_left[0]}, Y={self.joystick_left[1]}, "
                f"按键={self.joystick_left[2]}, 板机={self.joystick_left[3]}\n"
                f"右摇杆: X={self.joystick_right[0]}, Y={self.joystick_right[1]}, "
                f"按键={self.joystick_right[2]}, 板机={self.joystick_right[3]}\n"
                f"左臂关节(原始): {self.arm_joint_left}\n"
                f"右臂关节(原始): {self.arm_joint_right}\n"
                f"左臂关节(弧度): {[f'{x:.4f}' for x in self.arm_joint_left_rad]}\n"
                f"右臂关节(弧度): {[f'{x:.4f}' for x in self.arm_joint_right_rad]}\n"
                f"躯干IMU Acc: {[f'{x:.3f}' for x in self.torso_acc]} Gyro: {[f'{x:.3f}' for x in self.torso_gyro]} Quat: {[f'{x:.3f}' for x in self.torso_quat]}\n"
                f"附加IMU Acc: {[f'{x:.3f}' for x in self.extra_acc]} Gyro: {[f'{x:.3f}' for x in self.extra_gyro]} Quat: {[f'{x:.3f}' for x in self.extra_quat]}")

    def _convert_encoder_to_radian(self, encoder_value: int) -> float:
        """
        将编码器原始数据转换为弧度制
        
        Args:
            encoder_value: 编码器原始值
            
        Returns:
            float: 转换后的弧度值
        """
        return encoder_value * ENCODER_TO_RADIAN_RATIO

class RemoteManipulatorReader:
    def __init__(self, port: str = "/dev/tty.usbmodem326A335732351", baudrate: int = 2000000, 
                 queue_size: int = 10, enable_logging: bool = True):
        """
        初始化远程操作器数据读取器
        
        Args:
            port: 串口设备名称
            baudrate: 波特率
            queue_size: 数据队列大小
            enable_logging: 是否启用数据记录到文件
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.read_thread = None
        
        # 使用固定大小的队列，当队列满时自动丢弃旧数据
        self.data_queue = queue.Queue(maxsize=queue_size)
        self.latest_data = RemoteManipulatorData()
        
        # 回调函数列表
        self.callbacks = []
        
        # 回调处理线程
        self.callback_thread = None
        self.callback_lock = threading.Lock()
        
        # 数据记录相关
        self.enable_logging = enable_logging
        self.log_file = None
        self.csv_writer = None
        self.logged_data = []  # 存储所有记录的数据用于后续分析
        self.log_lock = threading.Lock()
        
        # 帧处理（对齐 SerialAdapter.js 的健壮拼帧方案）
        # 支持的完整帧长度：header(1) + payload(48/88/128) + checksum(1) + tail(1) = 51/91/131
        self.frame_header = REPORT_FRAME_HEADER
        self.frame_tail = REPORT_FRAME_TAIL
        self.supported_frame_lengths = [1 + DATA_LEN_BASE + 2,
                                        1 + DATA_LEN_TORSO_IMU_QUAT + 2,
                                        1 + DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT + 2]  # 51/91/131
        self.frame_buffer = bytearray()
        
        # 串口热插拔与自动重连
        self.reconnect_enabled = True
        self.reconnect_interval = 1.0  # 秒
        self._last_reconnect_attempt = 0.0
        self.serial_lock = threading.Lock()

        # 统计信息（解析帧数/速率/丢包同步）
        self.frames_parsed = 0
        self.frames_bad_sync = 0
        self.bytes_read = 0
        self.first_frame_time = None
        self.last_frame_time = None
        self.enable_stats = False
        self.stats_interval = 1.0
        self._last_stats_time = time.monotonic()
        self._frames_since_stats = 0

        # 日志刷新节流，避免频繁 flush 造成卡顿
        self._flush_every_n = 50
        self._log_count_since_flush = 0
        self._last_flush_time = time.monotonic()
        self._flush_interval = 0.5
        
        if self.enable_logging:
            self._setup_logging()
    
    def _setup_logging(self):
        """设置数据记录"""
        try:
            # 创建日志文件名（包含时间戳）
            script_dir = os.path.dirname(os.path.abspath(__file__))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_filename = f"remote_manipulator_data_{timestamp}.csv"
            self.log_filepath = os.path.join(script_dir, log_filename)
            
            # 创建CSV文件并写入标题行
            self.log_file = open(self.log_filepath, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.log_file)
            
            # 写入CSV标题行
            headers = ['timestamp', 'relative_time', 'format_version']
            # 摇杆数据
            for side in ['left', 'right']:
                for param in ['x', 'y', 'button', 'trigger']:
                    headers.append(f'joystick_{side}_{param}')
            # 关节数据 - 原始值
            for side in ['left', 'right']:
                for i in range(8):
                    headers.append(f'arm_joint_{side}_raw_{i+1}')
            # 关节数据 - 弧度值
            for side in ['left', 'right']:
                for i in range(8):
                    headers.append(f'arm_joint_{side}_rad_{i+1}')
            # 躯干IMU
            for comp in ['acc', 'gyro', 'quat']:
                for axis in ['x', 'y', 'z']:
                    if comp == 'quat':
                        # 四元数4分量单独处理，不使用xyz
                        break
                    headers.append(f'torso_{comp}_{axis}')
            # 四元数分量
            for comp in ['q0', 'q1', 'q2', 'q3']:
                headers.append(f'torso_quat_{comp}')
            # 附加IMU（头部等）
            for comp in ['acc', 'gyro', 'quat']:
                for axis in ['x', 'y', 'z']:
                    if comp == 'quat':
                        break
                    headers.append(f'extra_{comp}_{axis}')
            for comp in ['q0', 'q1', 'q2', 'q3']:
                headers.append(f'extra_quat_{comp}')
            
            self.csv_writer.writerow(headers)
            self.start_time = None  # 记录开始时间
            
            logger.info(f"数据记录已启用，文件路径: {self.log_filepath}")
            
        except Exception as e:
            logger.error(f"设置数据记录失败: {str(e)}")
            self.enable_logging = False
    
    def _log_data(self, data: RemoteManipulatorData):
        """记录数据到CSV文件"""
        if not self.enable_logging or not self.csv_writer:
            return
            
        try:
            with self.log_lock:
                # 设置开始时间
                if self.start_time is None:
                    self.start_time = data.timestamp
                
                relative_time = data.timestamp - self.start_time
                
                # 准备CSV行数据
                row_data = [data.timestamp, relative_time, data.format_version]
                
                # 添加摇杆数据
                row_data.extend(data.joystick_left)
                row_data.extend(data.joystick_right)
                
                # 添加关节原始数据
                row_data.extend(data.arm_joint_left)
                row_data.extend(data.arm_joint_right)
                
                # 添加关节弧度数据
                row_data.extend(data.arm_joint_left_rad)
                row_data.extend(data.arm_joint_right_rad)
                
                # 添加躯干IMU（固定三轴）
                row_data.extend([data.torso_acc[0], data.torso_acc[1], data.torso_acc[2]])
                row_data.extend([data.torso_gyro[0], data.torso_gyro[1], data.torso_gyro[2]])
                # 躯干四元数
                row_data.extend([data.torso_quat[0], data.torso_quat[1], data.torso_quat[2], data.torso_quat[3]])
                
                # 添加附加IMU
                row_data.extend([data.extra_acc[0], data.extra_acc[1], data.extra_acc[2]])
                row_data.extend([data.extra_gyro[0], data.extra_gyro[1], data.extra_gyro[2]])
                row_data.extend([data.extra_quat[0], data.extra_quat[1], data.extra_quat[2], data.extra_quat[3]])
                
                # 写入CSV文件
                self.csv_writer.writerow(row_data)
                self._log_count_since_flush += 1
                now = time.monotonic()
                if (self._log_count_since_flush >= self._flush_every_n or
                        (now - self._last_flush_time) >= self._flush_interval):
                    self.log_file.flush()
                    self._log_count_since_flush = 0
                    self._last_flush_time = now
                
                # 存储数据用于后续分析
                self.logged_data.append({
                    'timestamp': data.timestamp,
                    'relative_time': relative_time,
                    'joystick_left': data.joystick_left.copy(),
                    'joystick_right': data.joystick_right.copy(),
                    'arm_joint_left_raw': data.arm_joint_left.copy(),
                    'arm_joint_right_raw': data.arm_joint_right.copy(),
                    'arm_joint_left_rad': data.arm_joint_left_rad.copy(),
                    'arm_joint_right_rad': data.arm_joint_right_rad.copy()
                })
                
        except Exception as e:
            logger.error(f"记录数据失败: {str(e)}")

    def start(self) -> bool:
        """
        启动数据读取
        
        Returns:
            bool: 是否成功启动
        """
        if self.running:
            logger.warning("数据读取器已经在运行")
            return True
        
        try:
            opened = self._open_serial()
            self.running = True
            
            # 启动数据读取线程
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            # 启动回调处理线程
            self.callback_thread = threading.Thread(target=self._callback_loop, daemon=True)
            self.callback_thread.start()
            
            if opened:
                logger.info(f"成功启动数据读取，端口：{self.port}, 波特率：{self.baudrate}")
            else:
                logger.warning(f"启动成功但串口未就绪，将自动重连，目标端口：{self.port}")
            return True
        except Exception as e:
            logger.error(f"启动数据读取失败: {str(e)}")
            self.running = False
            self._close_serial()
            return False
    
    def stop(self) -> None:
        """停止数据读取"""
        self.running = False
        
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
            
        if self.callback_thread:
            self.callback_thread.join(timeout=1.0)
            
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        # 关闭日志文件
        if self.log_file:
            self.log_file.close()
            
        logger.info("数据读取已停止")
        
        # 如果启用了记录且有数据，则绘制图表
        if self.enable_logging and self.logged_data:
            self.plot_analysis_charts()
    
    def _read_loop(self) -> None:
        """数据读取循环（采用健壮的帧累积与同步方案）"""
        while self.running:
            try:
                # 优先检查并维护连接
                ser_ref = None
                with self.serial_lock:
                    ser_ref = self.serial
                if not ser_ref or not getattr(ser_ref, "is_open", False):
                    self._try_reconnect()
                    time.sleep(0.1)
                    continue
                
                # 读取数据
                try:
                    in_bytes = ser_ref.in_waiting
                except Exception as e:
                    logger.error(f"串口状态异常，准备重连: {str(e)}")
                    self._close_serial()
                    time.sleep(0.2)
                    continue
                
                # 在部分平台（尤其是 macOS）上，in_waiting 可能长期为0，但带超时的读取可正常获得数据
                try:
                    if in_bytes and in_bytes > 0:
                        chunk = ser_ref.read(min(in_bytes, 4096))
                    else:
                        # 触发一次短超时阻塞读，避免因 in_waiting 恒为0 而不读取缓冲区
                        chunk = ser_ref.read(1)
                        if chunk:
                            # 若已读到首字节，尽量把缓冲区剩余数据一次性取完
                            try:
                                more = ser_ref.read(ser_ref.in_waiting)
                            except Exception:
                                more = b""
                            if more:
                                chunk += more
                except Exception as e:
                    logger.error(f"串口读取异常，准备重连: {str(e)}")
                    self._close_serial()
                    time.sleep(0.2)
                    continue
                if chunk:
                    self.bytes_read += len(chunk)
                    self._accumulate_and_process_frames(chunk)
                else:
                    time.sleep(0.001)
                # 防止CPU占用过高
                time.sleep(0.0002)
            except Exception as e:
                logger.error(f"数据读取错误: {str(e)}")
                time.sleep(1)  # 发生错误时暂停一下
    
    def _open_serial(self) -> bool:
        """尝试打开串口（支持热插拔重连调用）"""
        try:
            with self.serial_lock:
                # 已打开则复用
                if self.serial and getattr(self.serial, "is_open", False):
                    return True
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    # 使用小超时，保证在 macOS 上即使 in_waiting 为0 也能读到数据
                    timeout=0.01,
                    write_timeout=0
                )
            logger.info(f"串口已打开：{self.port}")
            return True
        except Exception as e:
            logger.warning(f"打开串口失败：{self.port}，错误：{str(e)}")
            with self.serial_lock:
                self.serial = None
            return False
    
    def _close_serial(self) -> None:
        """安全关闭串口"""
        with self.serial_lock:
            try:
                if self.serial and getattr(self.serial, "is_open", False):
                    self.serial.close()
            except Exception:
                pass
            finally:
                self.serial = None
    
    def _try_reconnect(self) -> None:
        """按节流策略尝试重连串口"""
        if not self.reconnect_enabled:
            return
        now = time.time()
        if now - self._last_reconnect_attempt < self.reconnect_interval:
            return
        self._last_reconnect_attempt = now
        if self._open_serial():
            logger.info(f"串口重连成功：{self.port}")

    def _accumulate_and_process_frames(self, chunk: bytes) -> None:
        """
        累积串口原始数据并按帧输出（参考 SerialAdapter.js 的实现）
        支持帧长度：51/91/131（分别对应payload: 48/88/128）
        """
        if not chunk:
            return
        # 累积到拼帧缓冲
        self.frame_buffer.extend(chunk)
        min_len = min(self.supported_frame_lengths)  # 51
        # 持续尝试从缓冲中提取有效帧
        while len(self.frame_buffer) >= min_len:
            # 查找帧头
            header_index = self.frame_buffer.find(bytes([self.frame_header]))
            if header_index == -1:
                # 无帧头，清空缓冲避免无限增长
                self.frame_buffer.clear()
                break
            if header_index > 0:
                # 丢弃帧头之前的噪声
                self.frame_buffer = self.frame_buffer[header_index:]
            # 若长度不足最小帧长，等待更多数据
            if len(self.frame_buffer) < min_len:
                break
            parsed = False
            for frame_len in self.supported_frame_lengths:
                if len(self.frame_buffer) < frame_len:
                    continue
                candidate = self.frame_buffer[:frame_len]
                # 帧尾校验
                if candidate[-1] != self.frame_tail:
                    continue
                # 异或校验（不含帧头和帧尾，校验值在倒数第二字节）
                checksum_calc = 0
                for i in range(1, frame_len - 2):
                    checksum_calc ^= candidate[i]
                if checksum_calc != candidate[frame_len - 2]:
                    continue
                # 转为载荷长度，兼容现有解析函数
                payload_len = frame_len - 3  # 去掉header/checksum/tail
                if self._parse_data_packet(candidate, payload_len):
                    # 入队供回调线程处理
                    try:
                        if self.data_queue.full():
                            self.data_queue.get_nowait()
                        self.data_queue.put_nowait(self.latest_data)
                    except queue.Full:
                        pass
                    # 消费该帧
                    self.frame_buffer = self.frame_buffer[frame_len:]
                    parsed = True
                    break
            if not parsed:
                # 无法匹配有效帧，丢弃1字节继续同步
                self.frame_buffer = self.frame_buffer[1:]
                self.frames_bad_sync += 1
    
    def _parse_data_packet(self, packet: bytearray, data_len: int) -> bool:
        """
        解析数据包
        
        Args:
            packet: 完整的数据包
            data_len: 载荷长度（不含帧头/校验/帧尾）
            
        Returns:
            bool: 是否成功解析
        """
        try:
            # 跳过帧头，直接解析数据部分
            data_part = packet[1:-2]  # 去掉帧头、校验和和帧尾
            
            # 创建新的数据对象
            data = RemoteManipulatorData()
            data.timestamp = time.time()
            
            # 解析左摇杆数据 (4个int16_t = 8字节)
            offset = 0
            for i in range(4):
                data.joystick_left[i] = struct.unpack('<h', data_part[offset:offset+2])[0]
                offset += 2
                
            # 解析右摇杆数据 (4个int16_t = 8字节)
            for i in range(4):
                data.joystick_right[i] = struct.unpack('<h', data_part[offset:offset+2])[0]
                offset += 2
                
            # 解析左臂关节数据 (8个int16_t = 16字节)
            for i in range(8):
                raw_value = struct.unpack('<h', data_part[offset:offset+2])[0]
                data.arm_joint_left[i] = raw_value
                # 转换为弧度制
                data.arm_joint_left_rad[i] = data._convert_encoder_to_radian(raw_value)
                offset += 2
                
            # 解析右臂关节数据 (8个int16_t = 16字节)
            for i in range(8):
                raw_value = struct.unpack('<h', data_part[offset:offset+2])[0]
                data.arm_joint_right[i] = raw_value
                # 转换为弧度制
                data.arm_joint_right_rad[i] = data._convert_encoder_to_radian(raw_value)
                offset += 2

            # 旧版基础数据长度为48字节；新版四元数格式长度为88/128字节
            if data_len == DATA_LEN_BASE:
                data.format_version = 1
            elif data_len == DATA_LEN_TORSO_IMU_QUAT:
                # 躯干IMU(四元数): fAcc[3], fGyro[3], fQuat[4]
                data.format_version = 2
                for i in range(3):
                    data.torso_acc[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.torso_gyro[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(4):
                    data.torso_quat[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
            elif data_len == DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT:
                # 躯干IMU + 附加IMU (四元数)
                data.format_version = 3
                for i in range(3):
                    data.torso_acc[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.torso_gyro[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(4):
                    data.torso_quat[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.extra_acc[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.extra_gyro[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(4):
                    data.extra_quat[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
            else:
                # 不支持的数据长度
                return False

            # 更新最新数据
            self.latest_data = data
            self.frames_parsed += 1
            self._frames_since_stats += 1
            if self.first_frame_time is None:
                self.first_frame_time = data.timestamp
            self.last_frame_time = data.timestamp
            return True
            
        except Exception as e:
            logger.error(f"数据解析错误: {str(e)}")
            return False
    
    def _callback_loop(self) -> None:
        """回调处理循环"""
        while self.running:
            try:
                # 从队列获取最新数据
                if not self.data_queue.empty():
                    data = self.data_queue.get_nowait()
                    
                    # 记录数据到文件
                    if self.enable_logging:
                        self._log_data(data)
                    
                    # 调用所有注册的回调函数
                    with self.callback_lock:
                        for callback in self.callbacks:
                            try:
                                callback(data)
                            except Exception as e:
                                logger.error(f"回调函数执行错误: {str(e)}")

                # 统计信息输出（可选）
                if self.enable_stats:
                    now = time.monotonic()
                    if now - self._last_stats_time >= self.stats_interval:
                        dt = now - self._last_stats_time
                        fps = self._frames_since_stats / dt if dt > 0 else 0.0
                        logger.info(
                            f"RX fps={fps:.1f}, parsed={self.frames_parsed}, "
                            f"bad_sync={self.frames_bad_sync}, bytes={self.bytes_read}"
                        )
                        self._frames_since_stats = 0
                        self._last_stats_time = now
                
                # 防止CPU占用过高
                time.sleep(0.0002)
                
            except queue.Empty:
                time.sleep(0.0002)  # 队列为空时等待
            except Exception as e:
                logger.error(f"回调处理错误: {str(e)}")
                time.sleep(1)  # 发生错误时暂停一下
    
    def get_latest_data(self) -> RemoteManipulatorData:
        """
        获取最新的传感器数据
        
        Returns:
            RemoteManipulatorData: 最新的数据
        """
        return self.latest_data
    
    def register_callback(self, callback: Callable[[RemoteManipulatorData], None]) -> bool:
        """
        注册数据回调函数
        
        Args:
            callback: 回调函数，接收一个RemoteManipulatorData参数
            
        Returns:
            bool: 是否成功注册
        """
        with self.callback_lock:
            if callback not in self.callbacks:
                self.callbacks.append(callback)
                logger.info("已注册回调函数")
                return True
            else:
                logger.warning("回调函数已经注册")
                return False
    
    def unregister_callback(self, callback: Callable[[RemoteManipulatorData], None]) -> bool:
        """
        取消注册数据回调函数
        
        Args:
            callback: 之前注册的回调函数
            
        Returns:
            bool: 是否成功取消注册
        """
        with self.callback_lock:
            if callback in self.callbacks:
                self.callbacks.remove(callback)
                logger.info("已取消注册回调函数")
                return True
            else:
                logger.warning("回调函数未注册")
                return False

    def plot_analysis_charts(self):
        """绘制分析图表"""
        if not self.logged_data:
            logger.warning("没有记录的数据，无法绘制图表")
            return
            
        try:
            _require_analysis_dependencies()
            logger.info("开始绘制分析图表...")
            
            # 提取数据
            times = [d['relative_time'] for d in self.logged_data]
            
            # 创建图表目录
            script_dir = os.path.dirname(os.path.abspath(__file__))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            charts_dir = os.path.join(script_dir, f"charts_{timestamp}")
            os.makedirs(charts_dir, exist_ok=True)
            
            # 1. 绘制左臂关节位置图
            self._plot_joint_positions(times, 'left', charts_dir)
            
            # 2. 绘制右臂关节位置图
            self._plot_joint_positions(times, 'right', charts_dir)
            
            # 3. 绘制左臂关节速度图
            self._plot_joint_velocities(times, 'left', charts_dir)
            
            # 4. 绘制右臂关节速度图
            self._plot_joint_velocities(times, 'right', charts_dir)
            
            # 5. 绘制摇杆数据图
            self._plot_joystick_data(times, charts_dir)
            
            logger.info(f"图表已保存到目录: {charts_dir}")
            
        except Exception as e:
            logger.error(f"绘制图表失败: {str(e)}")
    
    def _plot_joint_positions(self, times, side, charts_dir):
        """绘制关节位置图"""
        fig, axes = plt.subplots(4, 2, figsize=(15, 12))
        fig.suptitle(f'{side.capitalize()} Arm Joint Positions (Radians)', fontsize=16)
        
        for i in range(8):
            row = i // 2
            col = i % 2
            
            positions = [d[f'arm_joint_{side}_rad'][i] for d in self.logged_data]
            
            axes[row, col].plot(times, positions, 'b-', linewidth=1)
            axes[row, col].set_title(f'Joint {i+1} Position')
            axes[row, col].set_xlabel('Time (s)')
            axes[row, col].set_ylabel('Position (rad)')
            axes[row, col].grid(True, alpha=0.3)
            
            # 添加统计信息
            mean_pos = np.mean(positions)
            std_pos = np.std(positions)
            axes[row, col].text(0.02, 0.98, f'Mean: {mean_pos:.4f}\nStd: {std_pos:.4f}', 
                              transform=axes[row, col].transAxes, verticalalignment='top',
                              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        plt.savefig(os.path.join(charts_dir, f'{side}_arm_joint_positions.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_joint_velocities(self, times, side, charts_dir):
        """绘制关节速度图"""
        fig, axes = plt.subplots(4, 2, figsize=(15, 12))
        fig.suptitle(f'{side.capitalize()} Arm Joint Velocities (rad/s)', fontsize=16)
        
        for i in range(8):
            row = i // 2
            col = i % 2
            
            positions = [d[f'arm_joint_{side}_rad'][i] for d in self.logged_data]
            
            # 计算速度（位置的导数）
            velocities = []
            for j in range(1, len(positions)):
                dt = times[j] - times[j-1]
                if dt > 0:
                    velocity = (positions[j] - positions[j-1]) / dt
                    velocities.append(velocity)
                else:
                    velocities.append(0.0)
            
            # 时间轴也需要对应调整
            vel_times = times[1:]
            
            if velocities:
                axes[row, col].plot(vel_times, velocities, 'r-', linewidth=1)
                axes[row, col].set_title(f'Joint {i+1} Velocity')
                axes[row, col].set_xlabel('Time (s)')
                axes[row, col].set_ylabel('Velocity (rad/s)')
                axes[row, col].grid(True, alpha=0.3)
                
                # 添加统计信息
                mean_vel = np.mean(velocities)
                std_vel = np.std(velocities)
                max_vel = np.max(np.abs(velocities))
                axes[row, col].text(0.02, 0.98, f'Mean: {mean_vel:.4f}\nStd: {std_vel:.4f}\nMax: {max_vel:.4f}', 
                                  transform=axes[row, col].transAxes, verticalalignment='top',
                                  bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
        plt.tight_layout()
        plt.savefig(os.path.join(charts_dir, f'{side}_arm_joint_velocities.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_joystick_data(self, times, charts_dir):
        """绘制摇杆数据图"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Joystick Data', fontsize=16)
        
        sides = ['left', 'right']
        for side_idx, side in enumerate(sides):
            # X轴数据
            x_data = [d[f'joystick_{side}'][0] for d in self.logged_data]
            axes[side_idx, 0].plot(times, x_data, 'g-', linewidth=1)
            axes[side_idx, 0].set_title(f'{side.capitalize()} Joystick X-axis')
            axes[side_idx, 0].set_xlabel('Time (s)')
            axes[side_idx, 0].set_ylabel('Value')
            axes[side_idx, 0].grid(True, alpha=0.3)
            
            # Y轴数据
            y_data = [d[f'joystick_{side}'][1] for d in self.logged_data]
            axes[side_idx, 1].plot(times, y_data, 'b-', linewidth=1)
            axes[side_idx, 1].set_title(f'{side.capitalize()} Joystick Y-axis')
            axes[side_idx, 1].set_xlabel('Time (s)')
            axes[side_idx, 1].set_ylabel('Value')
            axes[side_idx, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(charts_dir, 'joystick_data.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def get_logged_data(self) -> List[Dict]:
        """获取所有记录的数据"""
        return self.logged_data.copy()
    
    def save_analysis_summary(self):
        """保存分析摘要"""
        if not self.logged_data:
            return
            
        try:
            _require_analysis_dependencies()
            script_dir = os.path.dirname(os.path.abspath(__file__))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            summary_file = os.path.join(script_dir, f"analysis_summary_{timestamp}.txt")
            
            with open(summary_file, 'w', encoding='utf-8') as f:
                f.write("Remote Manipulator Data Analysis Summary\n")
                f.write("=" * 50 + "\n\n")
                
                f.write(f"Recording Duration: {self.logged_data[-1]['relative_time']:.2f} seconds\n")
                f.write(f"Total Data Points: {len(self.logged_data)}\n")
                f.write(f"Average Sample Rate: {len(self.logged_data) / self.logged_data[-1]['relative_time']:.2f} Hz\n\n")

                # 解析层面帧率（更接近真实接收）
                if self.first_frame_time and self.last_frame_time and self.last_frame_time > self.first_frame_time:
                    parse_rate = self.frames_parsed / (self.last_frame_time - self.first_frame_time)
                    f.write(f"Parsed Frame Rate: {parse_rate:.2f} Hz\n")
                    f.write(f"Parsed Frames: {self.frames_parsed}\n")
                    f.write(f"Bad Sync Drops: {self.frames_bad_sync}\n\n")
                
                # 分析每个关节的统计信息
                for side in ['left', 'right']:
                    f.write(f"{side.capitalize()} Arm Joint Statistics:\n")
                    f.write("-" * 30 + "\n")
                    
                    for i in range(8):
                        positions = [d[f'arm_joint_{side}_rad'][i] for d in self.logged_data]
                        mean_pos = np.mean(positions)
                        std_pos = np.std(positions)
                        min_pos = np.min(positions)
                        max_pos = np.max(positions)
                        
                        f.write(f"Joint {i+1}: Mean={mean_pos:.4f}, Std={std_pos:.4f}, "
                               f"Min={min_pos:.4f}, Max={max_pos:.4f}\n")
                    
                    f.write("\n")
                
            logger.info(f"分析摘要已保存到: {summary_file}")
            
        except Exception as e:
            logger.error(f"保存分析摘要失败: {str(e)}")


def run_gui_demo():
    """
    运行GUI演示程序
    
    注意：此函数需要安装以下依赖：
    - matplotlib
    - PyQt5
    - numpy
    
    可以通过以下命令安装：
    pip install matplotlib PyQt5 numpy
    """
    try:
        import sys
        import numpy as np
        import matplotlib
        matplotlib.use('Qt5Agg')
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
        from matplotlib.figure import Figure
        from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                                    QWidget, QTableWidget, QTableWidgetItem, QHeaderView, 
                                    QLabel, QSplitter)
        from PyQt5.QtCore import QTimer, Qt
    except ImportError as e:
        print(f"错误：缺少GUI依赖包，请安装：pip install matplotlib PyQt5 numpy")
        print(f"具体错误：{e}")
        return
    
    class RemoteManipulatorGUI(QMainWindow):
        def __init__(self):
            super().__init__()
            
            # 设置窗口标题和大小
            self.setWindowTitle("远程操作器右臂关节数据监视器")
            self.setGeometry(100, 100, 1200, 800)
            
            # 创建中央控件
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            main_layout = QHBoxLayout(central_widget)
            
            # 创建分割器
            splitter = QSplitter(Qt.Horizontal)
            main_layout.addWidget(splitter)
            
            # 创建左侧和右侧窗口部件
            left_widget = QWidget()
            right_widget = QWidget()
            left_layout = QVBoxLayout(left_widget)
            right_layout = QVBoxLayout(right_widget)
            
            # 添加到分割器
            splitter.addWidget(left_widget)
            splitter.addWidget(right_widget)
            splitter.setSizes([400, 800])  # 设置初始分割比例
            
            # 创建表格标题
            table_label = QLabel("右臂关节数据表格 (原始值 | 弧度值)")
            table_label.setAlignment(Qt.AlignCenter)
            left_layout.addWidget(table_label)
            
            # 创建数据表格
            self.table = QTableWidget()
            self.table.setColumnCount(17)  # 时间 + 8个关节原始值 + 8个关节弧度值
            self.table.setRowCount(20)    # 显示最近20条数据
            
            # 设置表格标题
            headers = ["时间(s)"]
            # 添加原始值列标题
            for i in range(8):
                headers.append(f"关节{i+1}(原始)")
            # 添加弧度值列标题
            for i in range(8):
                headers.append(f"关节{i+1}(弧度)")
            self.table.setHorizontalHeaderLabels(headers)
            
            # 设置表格列宽自动调整
            header = self.table.horizontalHeader()
            for i in range(17):
                header.setSectionResizeMode(i, QHeaderView.Stretch)
            
            left_layout.addWidget(self.table)
            
            # 创建图表标题
            chart_label = QLabel("右臂关节数据曲线")
            chart_label.setAlignment(Qt.AlignCenter)
            right_layout.addWidget(chart_label)
            
            # 创建图表
            self.figure = Figure(figsize=(8, 8), dpi=100)
            self.canvas = FigureCanvas(self.figure)
            right_layout.addWidget(self.canvas)
            
            # 创建8个子图用于显示每个关节的数据
            self.axes = []
            for i in range(8):
                ax = self.figure.add_subplot(4, 2, i+1)
                ax.set_title(f"Joint{i+1}")
                ax.set_xlabel("Time(s)")
                ax.set_ylabel("Angle(rad)")  # 更新Y轴标签为弧度
                ax.grid(True)
                self.axes.append(ax)
            
            self.figure.tight_layout()
            
            # 数据存储
            self.time_data = []  # 时间数据
            self.joint_data = [[] for _ in range(8)]  # 8个关节的数据
            self.max_data_points = 100  # 最多保存100个数据点
            self.display_data = []  # 用于表格显示的数据
            
            # 创建读取器实例（启用数据记录）
            self.reader = RemoteManipulatorReader(port="/dev/tty.usbmodem326A335732351", baudrate=2000000, enable_logging=True)
            
            # 启动数据读取
            if not self.reader.start():
                print("错误：无法启动数据读取器")
                sys.exit(1)
            
            # 记录起始时间
            self.start_time = time.time()
            
            # 创建定时器，50ms更新一次(20Hz)
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_data)
            self.timer.start(50)
            
            print("GUI已启动，开始监视右臂关节数据...")
        
        def update_data(self):
            """更新数据并刷新GUI"""
            # 获取最新数据
            data = self.reader.get_latest_data()
            
            # 获取当前时间（相对于起始时间）
            current_time = time.time() - self.start_time
            
            # 更新数据存储
            self.time_data.append(current_time)
            for i in range(8):
                self.joint_data[i].append(data.arm_joint_right_rad[i])  # 使用弧度制数据用于图表显示
            
            # 保存这一条完整数据用于表格显示
            self.display_data.append({
                'time': current_time,
                'joints_raw': data.arm_joint_right.copy(),  # 原始数据
                'joints_rad': data.arm_joint_right_rad.copy()  # 弧度数据
            })
            
            # 限制数据点数量
            if len(self.time_data) > self.max_data_points:
                self.time_data = self.time_data[-self.max_data_points:]
                for i in range(8):
                    self.joint_data[i] = self.joint_data[i][-self.max_data_points:]
                self.display_data = self.display_data[-20:]  # 只保留最近20条用于表格显示
            
            # 更新表格
            self.update_table()
            
            # 更新图表
            self.update_charts()
        
        def update_table(self):
            """更新表格数据"""
            # 清空表格
            self.table.clearContents()
            
            # 倒序显示数据（最新的在上面）
            for row, data_item in enumerate(reversed(self.display_data)):
                if row >= 20:  # 最多显示20行
                    break
                    
                # 设置时间
                self.table.setItem(row, 0, QTableWidgetItem(f"{data_item['time']:.2f}"))
                
                # 设置原始关节数据 (列1-8)
                for joint, value in enumerate(data_item['joints_raw']):
                    self.table.setItem(row, joint + 1, QTableWidgetItem(f"{value}"))
                
                # 设置弧度关节数据 (列9-16)
                for joint, value in enumerate(data_item['joints_rad']):
                    self.table.setItem(row, joint + 9, QTableWidgetItem(f"{value:.4f}"))
        
        def update_charts(self):
            """更新图表"""
            # 更新每个关节的图表
            for i in range(8):
                self.axes[i].clear()
                self.axes[i].plot(self.time_data, self.joint_data[i], 'b-')
                self.axes[i].set_title(f"Joint{i+1}")
                self.axes[i].set_xlabel("Time(s)")
                self.axes[i].set_ylabel("Angle(rad)")  # 更新Y轴标签为弧度
                self.axes[i].grid(True)
                
                # 设置Y轴范围，使曲线更加明显
                if len(self.joint_data[i]) > 1:
                    y_min = min(self.joint_data[i])
                    y_max = max(self.joint_data[i])
                    
                    # 处理所有值相同的情况
                    if y_min == y_max:
                        y_min -= 0.1  # 对于弧度值，使用更小的余量
                        y_max += 0.1
                    else:
                        # 添加一点余量
                        margin = (y_max - y_min) * 0.1
                        y_min -= margin
                        y_max += margin
                    
                    self.axes[i].set_ylim(y_min, y_max)
                
                # 设置X轴范围，只显示最近的数据
                if len(self.time_data) > 0:
                    x_min = max(0, self.time_data[-1] - 10)  # 显示最近10秒的数据
                    x_max = self.time_data[-1]
                    self.axes[i].set_xlim(x_min, x_max)
            
            # 调整布局并重绘
            self.figure.tight_layout()
            self.canvas.draw()
        
        def closeEvent(self, event):
            """窗口关闭事件处理"""
            # 停止数据读取器
            if hasattr(self, 'reader'):
                print("正在停止数据读取并生成分析图表...")
                self.reader.stop()
                # 保存分析摘要
                self.reader.save_analysis_summary()
                print("分析图表和摘要已生成")
            event.accept()
    
    # 创建QT应用
    app = QApplication(sys.argv)
    window = RemoteManipulatorGUI()
    window.show()
    sys.exit(app.exec_())


def data_recording_demo():
    """
    数据记录演示程序
    专门用于记录数据并生成分析图表
    """
    print("启动远程操作器数据记录程序...")
    print("=" * 60)
    print("功能说明：")
    print("- 实时记录所有传感器数据到CSV文件")
    print("- 程序结束时自动生成以下分析图表：")
    print("  * 左臂/右臂关节位置曲线（弧度）")
    print("  * 左臂/右臂关节速度曲线（弧度/秒）")
    print("  * 摇杆数据曲线")
    print("- 生成详细的数据统计摘要")
    print("=" * 60)
    
    # 创建读取器实例（启用数据记录）
    reader = RemoteManipulatorReader(port="/dev/tty.usbmodem355B346632351", baudrate=2000000, enable_logging=True)
    
    # 启动数据读取
    if not reader.start():
        print("错误：无法启动数据读取器")
        return
    
    print("数据记录已启动...")
    print("按 Ctrl+C 结束记录并生成分析图表")
    print("-" * 60)
    
    try:
        start_time = time.time()
        last_update = start_time
        
        while True:
            current_time = time.time()
            
            # 每200ms更新一次状态
            if current_time - last_update >= 0.2:
                duration = current_time - start_time
                logged = reader.get_logged_data()
                data_count = len(logged)
                # 瞬时采样率（2秒平均）
                if data_count > 0:
                    last_rel_time = logged[-1]['relative_time']
                    window_start = max(0.0, last_rel_time - 2.0)
                    recent_count = 0
                    for d in reversed(logged):
                        if d['relative_time'] >= window_start:
                            recent_count += 1
                        else:
                            break
                    window_duration = min(2.0, last_rel_time) if last_rel_time > 0 else 0.0
                    inst_rate = (recent_count / window_duration) if window_duration > 0 else 0.0
                else:
                    inst_rate = 0.0
                print(f"记录时长: {duration:.1f}s | 数据点数: {data_count} | 瞬时采样率(2s均值): {inst_rate:.1f} Hz")
                last_update = current_time
            
            time.sleep(0.0002)
            
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("正在停止数据记录...")
        print("正在生成分析图表和摘要，请稍等...")
        
        reader.stop()
        reader.save_analysis_summary()
        
        print("=" * 60)
        print("数据记录程序已完成！")
        print("生成的文件：")
        print("- CSV数据文件：remote_manipulator_data_*.csv")
        print("- 图表目录：charts_*")
        print("- 分析摘要：analysis_summary_*.txt")
        print("=" * 60)


def simple_console_demo():
    """
    简单的控制台演示程序
    """
    print("启动远程操作器数据读取演示...")
    # print("功能说明：")
    # print("- 数据将自动记录到CSV文件")
    # print("- 程序结束时会自动生成分析图表")
    # print("- 图表包括关节位置、速度曲线和摇杆数据")
    print("-" * 50)
    
    # 创建读取器实例（启用数据记录）
    reader = RemoteManipulatorReader(port="/dev/tty.usbmodem355B346632351", baudrate=2000000, enable_logging=False)
    
    # 定义回调函数
    def on_data_received(data):
        print(f"\n收到新数据 (时间戳: {data.timestamp:.3f}):")
        print(f"  左摇杆: X={data.joystick_left[0]:4d}, Y={data.joystick_left[1]:4d}, 按键={data.joystick_left[2]}, 板机={data.joystick_left[3]}")
        print(f"  右摇杆: X={data.joystick_right[0]:4d}, Y={data.joystick_right[1]:4d}, 按键={data.joystick_right[2]}, 板机={data.joystick_right[3]}")
        print(f"  左臂关节(原始): {[f'{x:4d}' for x in data.arm_joint_left]}")
        print(f"  右臂关节(原始): {[f'{x:4d}' for x in data.arm_joint_right]}")
        print(f"  左臂关节(弧度): {[f'{x:7.4f}' for x in data.arm_joint_left_rad]}")
        print(f"  右臂关节(弧度): {[f'{x:7.4f}' for x in data.arm_joint_right_rad]}")
        # 新增: 打印IMU数据
        print(f"  躯干IMU Acc: {[f'{x:.3f}' for x in data.torso_acc]}, Gyro: {[f'{x:.3f}' for x in data.torso_gyro]}, Quat: {[f'{x:.3f}' for x in data.torso_quat]}")
        print(f"  附加IMU  Acc: {[f'{x:.3f}' for x in data.extra_acc]}, Gyro: {[f'{x:.3f}' for x in data.extra_gyro]}, Quat: {[f'{x:.3f}' for x in data.extra_quat]}")
        

    # 注册回调函数
    reader.register_callback(on_data_received)
    
    # 启动数据读取
    if not reader.start():
        print("错误：无法启动数据读取器")
        return
    
    print("数据读取已启动，按 Ctrl+C 退出...")
    # print("提示：程序结束时会自动生成分析图表和摘要文件")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n正在停止数据读取...")
        # print("正在生成分析图表，请稍等...")
        # reader.stop()
        
        # # 保存分析摘要
        # reader.save_analysis_summary()
        
        # print("演示程序已退出")
        # print("请查看同目录下的以下文件：")
        # print("- CSV数据文件：remote_manipulator_data_*.csv")
        # print("- 图表目录：charts_*")
        # print("- 分析摘要：analysis_summary_*.txt")


# 示例用法
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--gui":
            # 运行GUI演示
            run_gui_demo()
        elif sys.argv[1] == "--record":
            # 运行数据记录演示
            data_recording_demo()
        elif sys.argv[1] == "--help":
            print("远程操作器数据读取器")
            print("使用方法：")
            print("  python remote_manipulator_data_reader.py           # 控制台演示")
            print("  python remote_manipulator_data_reader.py --gui     # GUI演示")
            print("  python remote_manipulator_data_reader.py --record  # 数据记录模式")
            print("  python remote_manipulator_data_reader.py --help    # 显示帮助")
        else:
            print("未知参数，使用 --help 查看帮助")
    else:
        # 运行简单的控制台演示
        simple_console_demo()
