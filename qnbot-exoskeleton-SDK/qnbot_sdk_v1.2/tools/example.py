import time

from remote_manipulator_data_reader import RemoteManipulatorReader

# 创建读取器实例
reader = RemoteManipulatorReader(port="/dev/tty.usbmodemCMSIS_DAP2", baudrate=2000000)

# 读取数据
# 启动数据读取
if not reader.start():
    print("错误：无法启动数据读取器") 
    exit()

print("数据读取已启动，按 Ctrl+C 退出...")


try:
    while True:
        latest_data = reader.get_latest_data()
        print(f"主动获取的最新数据:\n{latest_data.arm_joint_right_rad}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n正在停止数据读取...")
    # 使用完成后停止读取
    reader.stop()

