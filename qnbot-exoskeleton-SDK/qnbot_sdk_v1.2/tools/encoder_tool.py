#!/usr/bin/env python3
import argparse
import queue
import threading
import sys
import time
from typing import List, Optional, Tuple
import select
import os

if os.name == "nt":
    import msvcrt
    tty = None
    termios = None
else:
    import tty
    import termios

try:
    import serial  # pyserial
    import serial.tools.list_ports as list_ports
except Exception as e:
    print("[ERROR] 需要安装 pyserial: pip install pyserial", file=sys.stderr)
    raise


# ANSI颜色代码（跨平台兼容）
class Color:
    """ANSI颜色代码，支持Mac/Linux/Windows 10+"""
    # 启用Windows颜色支持
    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
        except Exception:
            pass
    
    # 基础颜色
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    
    # 前景色
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    
    # 高亮前景色
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    
    @staticmethod
    def disable():
        """禁用颜色（用于不支持ANSI的终端）"""
        Color.RESET = ''
        Color.BOLD = ''
        Color.DIM = ''
        Color.BLACK = ''
        Color.RED = ''
        Color.GREEN = ''
        Color.YELLOW = ''
        Color.BLUE = ''
        Color.MAGENTA = ''
        Color.CYAN = ''
        Color.WHITE = ''
        Color.BRIGHT_RED = ''
        Color.BRIGHT_GREEN = ''
        Color.BRIGHT_YELLOW = ''
        Color.BRIGHT_BLUE = ''
        Color.BRIGHT_MAGENTA = ''
        Color.BRIGHT_CYAN = ''

# 检测终端是否支持颜色
if not sys.stdout.isatty() or os.getenv('NO_COLOR'):
    Color.disable()


def colored(text: str, color: str = '', bold: bool = False) -> str:
    """为文本添加颜色"""
    if not color:
        return text
    prefix = f"{Color.BOLD if bold else ''}{color}"
    return f"{prefix}{text}{Color.RESET}"


def print_ok(msg: str):
    """打印成功信息（绿色）"""
    print(colored(msg, Color.BRIGHT_GREEN))


def print_err(msg: str):
    """打印错误信息（红色）"""
    print(colored(msg, Color.BRIGHT_RED))


def print_warn(msg: str):
    """打印警告信息（黄色）"""
    print(colored(msg, Color.BRIGHT_YELLOW))


def print_info(msg: str):
    """打印提示信息（蓝色）"""
    print(colored(msg, Color.BRIGHT_CYAN))


FRAME_HEADER = 0x55
FRAME_TAIL = 0xAA
FRAME_MAX_SIZE = 16

# 功能码
FUNC_READ_REQ = 0x01
FUNC_READ_RESP = 0x01 | 0x04  # 0x05
FUNC_WRITE_REQ = 0x02
FUNC_WRITE_RESP = 0x02 | 0x04  # 0x06

# 寄存器地址
REG_ADDR_VERSION = 0x00  # 软件版本 (4B, BE)
REG_ADDR_DEVICE_ID = 0x01  # 设备ID (1B, 低4位)
REG_ADDR_ZERO_OFFSET = 0x02  # 零位偏移 (2B, BE)
REG_ADDR_ZERO_SET_CURRENT = 0x03  # 将当前位置设为零位（写入触发，无负载）
REG_ADDR_SERIAL_NUMBER = 0x04  # 序列号 (6B, BE, 只读, 12字符十六进制)

# 固件版本定义（格式：HW_MAJOR.HW_MINOR.FW_MAJOR.FW_MINOR）
FIRMWARE_VERSION_LATEST = 0x02020103  # 硬件2.2 固件2.2.1.3

# 广播地址
ADDR_BROADCAST = 0x00

def op_prefix(addr: int) -> str:
    """生成带颜色的操作前缀"""
    if addr == ADDR_BROADCAST:
        return colored("[BROADCAST]", Color.BRIGHT_MAGENTA, bold=True)
    else:
        return colored(f"[ID={addr}]", Color.BRIGHT_CYAN, bold=True)


def format_version(ver: int) -> str:
    """将版本号格式化为 HW_x.y FW_a.b 形式。
    
    版本号格式: [HW_MAJOR(1B)][HW_MINOR(1B)][FW_MAJOR(1B)][FW_MINOR(1B)]
    例如: 0x02020103 = 硬件2.2 固件2.2.1.3
    """
    hw_major = (ver >> 24) & 0xFF
    hw_minor = (ver >> 16) & 0xFF
    fw_major = (ver >> 8) & 0xFF
    fw_minor = ver & 0xFF
    return f"HW{hw_major}.{hw_minor} FW{hw_major}.{hw_minor}.{fw_major}.{fw_minor}"


def format_serial_number(data: bytes) -> str:
    """将序列号6字节数据格式化为12字符十六进制字符串。"""
    return "".join(f"{b:02X}" for b in data)


def detect_default_port() -> Optional[str]:
    """尝试自动检测USB转485串口设备。"""
    candidates = []
    for p in list_ports.comports():
        name = p.device
        if any(k in name for k in ["usbserial", "usbmodem", "ttyUSB", "ttyACM", "cu.usb"]):
            candidates.append(name)
    return candidates[0] if candidates else None


def list_all_ports() -> List[str]:
    ports: List[str] = []
    for p in list_ports.comports():
        ports.append(p.device)
    return ports


def pack_addr_func(addr: int, func: int) -> int:
    return ((addr & 0x0F) << 4) | (func & 0x0F)


def build_frame(addr: int, func: int, data: bytes) -> bytes:
    # 帧: header | addr_func | data... | xor | tail
    addr_func = pack_addr_func(addr, func)
    xor_val = addr_func
    for b in data:
        xor_val ^= b
    frame = bytes([FRAME_HEADER, addr_func]) + data + bytes([xor_val, FRAME_TAIL])
    if len(frame) > FRAME_MAX_SIZE:
        raise ValueError("frame too long")
    return frame


def try_parse_frames(buf: bytearray) -> List[bytes]:
    """从缓冲区中解析出尽可能多的完整帧，返回帧列表(原始字节)。"""
    frames = []
    i = 0
    while i + 4 <= len(buf):
        # 寻找帧头
        if buf[i] != FRAME_HEADER:
            i += 1
            continue
        # 最短长度 4
        # 寻找帧尾（不跨越FRAME_MAX_SIZE）
        end_limit = min(i + FRAME_MAX_SIZE, len(buf))
        tail_pos = -1
        for j in range(i + 4, end_limit + 1):
            if j <= len(buf) and j - i >= 4:
                # 潜在尾部在 j-1
                if buf[j - 1] == FRAME_TAIL:
                    tail_pos = j - 1
                    break
        if tail_pos == -1:
            break  # 等待更多数据
        frame = bytes(buf[i : tail_pos + 1])
        # 校验XOR
        addr_func = frame[1]
        data = frame[2:-2]
        xor_calc = addr_func
        for b in data:
            xor_calc ^= b
        xor_rx = frame[-2]
        if xor_calc == xor_rx:
            frames.append(frame)
            # 移除已解析部分
            del buf[: tail_pos + 1]
            i = 0
        else:
            # 异常，丢弃当前头，继续
            i += 1
            del buf[:i]
            i = 0
    return frames


def decode_frame(frame: bytes) -> Tuple[int, int, bytes]:
    """返回 (addr, func, data)"""
    if len(frame) < 4 or frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
        raise ValueError("invalid frame")
    addr_func = frame[1]
    addr = (addr_func >> 4) & 0x0F
    func = addr_func & 0x0F
    data = frame[2:-2]
    return addr, func, data


def send_and_wait_frames(ser: serial.Serial, req: bytes, timeout: float = 0.1) -> List[Tuple[int, int, bytes]]:
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    buf = bytearray()
    t0 = time.time()
    results: List[Tuple[int, int, bytes]] = []
    while time.time() - t0 < timeout:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            buf.extend(chunk)
            frames = try_parse_frames(buf)
            for fr in frames:
                results.append(decode_frame(fr))
        else:
            time.sleep(0.002)
    return results


def read_angle(ser: serial.Serial, dev_id: int, timeout: float = 0.05) -> Optional[Tuple[int, int]]:
    """读取角度，返回 (resp_id, angle)。若使用广播地址0，则返回第一个响应的设备ID与角度。"""
    req = build_frame(dev_id, FUNC_READ_REQ, b"")
    resps = send_and_wait_frames(ser, req, timeout)
    for (addr, func, data) in resps:
        if func == (FUNC_READ_RESP & 0x0F) and len(data) >= 2:
            ang = ((data[0] << 8) | data[1]) & 0x3FFF
            if dev_id == ADDR_BROADCAST:
                return (addr, ang)
            if addr == dev_id:
                return (addr, ang)
    return None


def read_register(ser: serial.Serial, addr: int, reg: int, timeout: float = 0.1) -> List[Tuple[int, bytes]]:
    req = build_frame(addr, FUNC_READ_REQ, bytes([reg]))
    resps = send_and_wait_frames(ser, req, timeout)
    out: List[Tuple[int, bytes]] = []
    for (raddr, func, data) in resps:
        if func == (FUNC_READ_RESP & 0x0F):
            out.append((raddr, data))
    return out


def write_register(ser: serial.Serial, addr: int, reg: int, payload: bytes, timeout: float = 0.1) -> List[int]:
    req = build_frame(addr, FUNC_WRITE_REQ, bytes([reg]) + payload)
    resps = send_and_wait_frames(ser, req, timeout)
    acks: List[int] = []
    for (raddr, func, data) in resps:
        if func == (FUNC_WRITE_RESP & 0x0F):
            acks.append(raddr)
    return acks


def choose_port_interactively(default_baud: int) -> Tuple[Optional[str], int]:
    ports = list_all_ports()
    print("可用串口：")
    if not ports:
        print("  (无)\n请插入设备或手动输入串口路径。")
    else:
        for idx, p in enumerate(ports):
            print(f"  [{idx}] {p}")
    while True:
        s = input("选择编号或输入设备路径 (回车刷新列表，q退出)：").strip()
        if s == "q":
            return None, default_baud
        if s == "":
            ports = list_all_ports()
            print("\n可用串口(刷新)：")
            if not ports:
                print("  (无)")
            else:
                for idx, p in enumerate(ports):
                    print(f"  [{idx}] {p}")
            continue
        if s.isdigit():
            i = int(s)
            if 0 <= i < len(ports):
                return ports[i], default_baud
            else:
                print("无效编号")
                continue
        # 视为路径
        return s, default_baud


def cmd_ui(args):
    # 交互式持续运行模式
    baud = args.baud or 2000000
    port = args.port
    if not port:
        port, baud = choose_port_interactively(baud)
        if not port:
            print("已退出")
            return 0

    def open_serial(p: str, b: int) -> Optional[serial.Serial]:
        try:
            return serial.Serial(port=p, baudrate=b, timeout=0)
        except Exception as e:
            print(f"[ERR] 打开串口失败: {e}")
            return None

    ser = open_serial(port, baud)
    if ser is None:
        return 1

    current_id = 1
    polling = False
    poll_interval = 0.05
    
    def print_menu():
        """打印精简菜单。"""
        title = colored("\n进入交互模式（精简）：", Color.BRIGHT_BLUE, bold=True)
        print(title)
        print(f"  {colored('2)', Color.YELLOW)} 写设备ID (定向)")
        print(f"  {colored('3)', Color.YELLOW)} 写零位 (定向)")
        print(f"  {colored('4)', Color.YELLOW)} 写设备ID (广播)")
        print(f"  {colored('5)', Color.YELLOW)} 写零位 (广播)")
        print(f"  {colored('1)', Color.YELLOW)} 轮询角度 开/关")
        print(f"  {colored('i)', Color.YELLOW)} 仅切换当前目标ID（不写入设备）")
        print(f"  {colored('m)', Color.YELLOW)} 显示高级菜单")
        print(f"  {colored('p)', Color.YELLOW)} 选择串口/波特率")
        print(f"  {colored('q)', Color.RED)} 退出")

    def print_advanced_menu():
        """打印高级菜单。"""
        title = colored("\n高级菜单：", Color.BRIGHT_BLUE, bold=True)
        print(title)
        print(f"  {colored('f 或 0)', Color.YELLOW)} 工厂模式（检查版本 -> 广播设ID -> 置零 -> 切换ID质检）")
        print(f"  {colored('r 或 6)', Color.YELLOW)} 读取寄存器（version/id/zero/serial/angle）")
        print(f"  {colored('z 或 7)', Color.YELLOW)} 将当前位置设为零位 (定向)")
        print(f"  {colored('b 或 8)', Color.YELLOW)} 将当前位置设为零位 (广播)")
        print(f"  {colored('s 或 9)', Color.YELLOW)} 读取序列号 (定向/广播)")
    
    print_menu()

    old_term = None
    quit_all = False
    last_factory_id: Optional[int] = None
    try:
        while True:
            if polling:
                # 进入轮询时设置终端为cbreak，支持按键即时停止
                if old_term is None and os.name != "nt":
                    try:
                        old_term = termios.tcgetattr(sys.stdin.fileno())
                        tty.setcbreak(sys.stdin.fileno())
                        print("\n[轮询中] 按 1 停止轮询，按 q 退出工具。", flush=True)
                    except Exception:
                        old_term = None
                elif old_term is None and os.name == "nt":
                    print("\n[轮询中] 按 1 停止轮询，按 q 退出工具。", flush=True)

                result = read_angle(ser, current_id, timeout=0.05)
                if result is not None:
                    rid, ang = result
                    label_id = rid if current_id == ADDR_BROADCAST else current_id
                    print(f"[ID={label_id}] 角度={ang:5d}", end="\r", flush=True)
                else:
                    label_id = '*' if current_id == ADDR_BROADCAST else current_id
                    print(f"[ID={label_id}] 角度=---- ", end="\r", flush=True)

                # 检查键盘
                try:
                    if os.name == "nt":
                        if msvcrt.kbhit():
                            ch = msvcrt.getwch()
                            if ch in ("\x00", "\xe0"):
                                # 功能键前缀，丢弃后续扫描码
                                if msvcrt.kbhit():
                                    msvcrt.getwch()
                            elif ch == '1':
                                polling = False
                            elif ch.lower() == 'q':
                                quit_all = True
                                polling = False
                        # 少量sleep防止占用CPU
                        time.sleep(0.01)
                    else:
                        r, _, _ = select.select([sys.stdin], [], [], 0.02)
                        if r:
                            ch = sys.stdin.read(1)
                            if ch == '1':
                                polling = False
                            elif ch.lower() == 'q':
                                quit_all = True
                                polling = False
                        # 少量sleep防止占用CPU
                        time.sleep(0.01)
                except Exception:
                    time.sleep(poll_interval)

                if ser.closed:
                    polling = False

                if not polling:
                    # 恢复终端设置
                    if old_term is not None and os.name != "nt":
                        try:
                            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
                        except Exception:
                            pass
                        old_term = None
                    print()  # 换行
                    if quit_all:
                        break
            else:
                prefix = op_prefix(current_id)
                cmd = input(f"\n{prefix} 选择操作(2/3/4/5/1/i/m/p/q)：").strip().lower()
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd in ("m", "menu"):
                    print_advanced_menu()
                elif cmd in ("1", "poll"):
                    polling = not polling
                    print(f"轮询={'开启' if polling else '关闭'} (ID={current_id})")
                elif cmd in ("2", "id"):
                    try:
                        nid = int(input("输入新设备ID(0~15)：").strip(), 0) & 0x0F
                        acks = write_register(ser, current_id, REG_ADDR_DEVICE_ID, bytes([nid]))
                        if acks:
                            print_ok(f"{op_prefix(current_id)} [OK] 设置设备ID={nid} 已确认")
                            current_id = nid
                        else:
                            print_err(f"{op_prefix(current_id)} [ERR] 无响应")
                    except Exception as e:
                        print_err(f"{op_prefix(current_id)} [ERR] {e}")
                elif cmd in ("3", "zero"):
                    try:
                        zv = int(input("输入零位(0~16383)：").strip(), 0) & 0x3FFF
                        payload = bytes([(zv >> 8) & 0xFF, zv & 0xFF])
                        acks = write_register(ser, current_id, REG_ADDR_ZERO_OFFSET, payload)
                        if acks:
                            print_ok(f"{op_prefix(current_id)} [OK] 设置零位成功")
                        else:
                            print_err(f"{op_prefix(current_id)} [ERR] 无响应")
                    except Exception as e:
                        print_err(f"{op_prefix(current_id)} [ERR] {e}")
                elif cmd in ("4", "bid"):
                    try:
                        nid = int(input("输入新设备ID(0~15)，广播写入：").strip(), 0) & 0x0F
                        acks = write_register(ser, ADDR_BROADCAST, REG_ADDR_DEVICE_ID, bytes([nid]))
                        if acks:
                            ids = ", ".join(str(a) for a in sorted(set(acks)))
                            print_ok(f"{op_prefix(ADDR_BROADCAST)} [OK] 响应设备: {ids}")
                        else:
                            print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] 无响应")
                    except Exception as e:
                        print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] {e}")
                elif cmd in ("5", "bzero"):
                    try:
                        zv = int(input("输入零位(0~16383)，广播写入：").strip(), 0) & 0x3FFF
                        payload = bytes([(zv >> 8) & 0xFF, zv & 0xFF])
                        acks = write_register(ser, ADDR_BROADCAST, REG_ADDR_ZERO_OFFSET, payload)
                        if acks:
                            ids = ", ".join(str(a) for a in sorted(set(acks)))
                            print_ok(f"{op_prefix(ADDR_BROADCAST)} [OK] 响应设备: {ids}")
                        else:
                            print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] 无响应")
                    except Exception as e:
                        print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] {e}")
                elif cmd in ("6", "r", "read"):
                    try:
                        scope = input("读取范围：b=广播, d=定向：").strip().lower()
                        regn = input("寄存器(version/id/zero/serial/angle)：").strip().lower()
                        addr = ADDR_BROADCAST if scope == "b" else current_id
                        if regn not in ("version", "id", "zero", "serial", "angle"):
                            print("无效寄存器")
                            continue
                        class A: pass
                        a = A()
                        a.port = port
                        a.baud = baud
                        a.id = addr
                        a.broadcast = (addr == ADDR_BROADCAST)
                        a.reg = regn
                        cmd_read(a)
                    except Exception as e:
                        print(f"{op_prefix(current_id if scope != 'b' else ADDR_BROADCAST)} [ERR] {e}")
                elif cmd in ("0", "f", "factory"):
                    # 工厂模式：输入目标ID(默认使用上次设置ID) -> 检查版本 -> 广播设ID -> 延时 -> 广播将当前位置设零 -> 延时 -> 切换目标ID轮询质检
                    try:
                        hint = f"{last_factory_id}" if last_factory_id is not None else "无"
                        tgt = input(f"输入目标设备ID(0~15)，上次ID={hint}，直接回车默认使用上次ID：").strip()
                        if tgt == "":
                            if last_factory_id is None:
                                print("未有上次ID，已取消")
                                continue
                            target_id = int(last_factory_id) & 0x0F
                        else:
                            target_id = int(tgt, 0) & 0x0F
                    except Exception as e:
                        print(f"[ERR] 目标ID无效: {e}")
                        continue
                    try:
                        # 步骤0：检查固件版本
                        print_info(f"{op_prefix(ADDR_BROADCAST)} [STEP] 广播检查固件版本")
                        ver_results = read_register(ser, ADDR_BROADCAST, REG_ADDR_VERSION, timeout=0.3)
                        if not ver_results:
                            print_warn(f"{op_prefix(ADDR_BROADCAST)} [WARN] 未检测到设备响应")
                            confirm = input("是否继续？(y/n): ").strip().lower()
                            if confirm != 'y':
                                continue
                        else:
                            version_ok = True
                            for (raddr, data) in ver_results:
                                if len(data) >= 4:
                                    ver = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
                                    ver_str = format_version(ver)
                                    latest_str = format_version(FIRMWARE_VERSION_LATEST)
                                    if ver < FIRMWARE_VERSION_LATEST:
                                        print_warn(f"{op_prefix(ADDR_BROADCAST)} [WARN] 设备ID={raddr} 版本={ver_str} < 最新={latest_str}")
                                        version_ok = False
                                    else:
                                        print_ok(f"{op_prefix(ADDR_BROADCAST)} [OK] 设备ID={raddr} 版本={ver_str}")
                            if not version_ok:
                                confirm = input("检测到旧版本固件，是否继续？(y/n): ").strip().lower()
                                if confirm != 'y':
                                    continue
                        
                        print_info(f"{op_prefix(ADDR_BROADCAST)} [STEP] 广播设置设备ID={target_id}")
                        acks = write_register(ser, ADDR_BROADCAST, REG_ADDR_DEVICE_ID, bytes([target_id]))
                        if not acks:
                            print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] 无响应(广播设ID)")
                            continue
                        time.sleep(0.2)
                        print_info(f"{op_prefix(ADDR_BROADCAST)} [STEP] 广播将当前位置设为零位")
                        acks2 = write_register(ser, ADDR_BROADCAST, REG_ADDR_ZERO_SET_CURRENT, b"")
                        if not acks2:
                            print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] 无响应(广播设零)")
                            continue
                        time.sleep(0.2)
                        current_id = target_id
                        last_factory_id = target_id
                        print_info(f"{op_prefix(current_id)} [STEP] 切换到目标ID={current_id} 进行质检(2秒)")
                        t0 = time.time()
                        ok_cnt = 0
                        while time.time() - t0 < 2.0:
                            result = read_angle(ser, current_id, timeout=0.05)
                            if result is not None:
                                _, ang = result
                                ok_cnt += 1
                                qc_text = f"[QC][ID={current_id}] 角度={ang:5d}"
                                print(colored(qc_text, Color.CYAN), end="\r", flush=True)
                            else:
                                qc_text = f"[QC][ID={current_id}] 角度=---- "
                                print(colored(qc_text, Color.DIM), end="\r", flush=True)
                            time.sleep(0.05)
                        print()  # 换行
                        if ok_cnt > 0:
                            print_ok("[OK] 质检完成")
                        else:
                            print_warn("[WARN] 质检期间未读到角度")
                        print_menu()  # 重新显示菜单
                    except Exception as e:
                        print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] 自动化流程失败: {e}")
                elif cmd in ("7", "z", "zero_here"):
                    # 定向：将当前位置设为零位
                    try:
                        acks = write_register(ser, current_id, REG_ADDR_ZERO_SET_CURRENT, b"")
                        if acks:
                            print_ok(f"{op_prefix(current_id)} [OK] 将当前位置设为零位成功")
                        else:
                            print_err(f"{op_prefix(current_id)} [ERR] 无响应")
                    except Exception as e:
                        print_err(f"{op_prefix(current_id)} [ERR] {e}")
                elif cmd in ("8", "b", "bzero_here"):
                    # 广播：将当前位置设为零位
                    try:
                        acks = write_register(ser, ADDR_BROADCAST, REG_ADDR_ZERO_SET_CURRENT, b"")
                        if acks:
                            ids = ", ".join(str(a) for a in sorted(set(acks)))
                            print_ok(f"{op_prefix(ADDR_BROADCAST)} [OK] 响应设备: {ids}")
                        else:
                            print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] 无响应")
                    except Exception as e:
                        print_err(f"{op_prefix(ADDR_BROADCAST)} [ERR] {e}")
                elif cmd == "p":
                    # 重新选择串口/波特率
                    try:
                        ser.close()
                    except Exception:
                        pass
                    port, baud = choose_port_interactively(baud)
                    if not port:
                        print("保持原连接")
                        ser = open_serial(port, baud)
                    else:
                        ser = open_serial(port, baud)
                        if ser is None:
                            print("[ERR] 连接失败，返回菜单")
                elif cmd == "i":
                    try:
                        nid = int(input(f"当前ID={current_id}，输入新目标ID(0~15)：").strip(), 0) & 0x0F
                        current_id = nid
                        print_ok(f"[OK] 切换脚本目标ID={current_id}")
                        print_menu()
                    except Exception as e:
                        print_err(f"[ERR] {e}")
                elif cmd in ("9", "s", "sn", "serial"):
                    # 读取序列号
                    try:
                        scope = input("读取范围：b=广播, d=定向：").strip().lower()
                        addr = ADDR_BROADCAST if scope == "b" else current_id
                        prefix = op_prefix(addr)
                        print_info(f"{prefix} 读取序列号...")
                        res = read_register(ser, addr, REG_ADDR_SERIAL_NUMBER, timeout=0.3)
                        if not res:
                            print_err(f"{prefix} [ERR] 无响应")
                        else:
                            for (raddr, data) in res:
                                if len(data) >= 6:
                                    sn = format_serial_number(data[:6])
                                    sn_colored = colored(sn, Color.BRIGHT_YELLOW, bold=True)
                                    print(f"{prefix} [RESP] from id={raddr}: SN={sn_colored}")
                                else:
                                    print_warn(f"{prefix} [RESP] from id={raddr}: 数据不足 ({len(data)}字节)")
                    except Exception as e:
                        print_err(f"{op_prefix(current_id if scope != 'b' else ADDR_BROADCAST)} [ERR] {e}")
                else:
                    print("未知操作")
    finally:
        # 恢复终端设置
        if old_term is not None and os.name != "nt":
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
            except Exception:
                pass
        try:
            ser.close()
        except Exception:
            pass
    return 0


def cmd_poll(args):
    port = args.port or detect_default_port()
    if not port:
        print("未找到串口，请使用 --port 指定，如 /dev/tty.usbserial-xxxx")
        return 1
    dev_id = args.id & 0x0F
    baud = args.baud
    interval = max(0.01, args.interval)
    print(f"[INFO] 打开串口 {port} @ {baud}，轮询设备ID={dev_id}")
    with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
        try:
            while True:
                result = read_angle(ser, dev_id, timeout=0.05)
                if result is not None:
                    rid, ang = result
                    if dev_id == ADDR_BROADCAST:
                        print(f"{op_prefix(dev_id)} ID={rid} 角度: {ang:5d}", end="\r", flush=True)
                    else:
                        print(f"{op_prefix(dev_id)} 角度: {ang:5d}", end="\r", flush=True)
                else:
                    print(f"{op_prefix(dev_id)} 角度: ---- ", end="\r", flush=True)
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n[INFO] 停止轮询")
    return 0


def cmd_read(args):
    port = args.port or detect_default_port()
    if not port:
        print("未找到串口，请使用 --port 指定")
        return 1
    addr = ADDR_BROADCAST if args.broadcast else (args.id & 0x0F)
    reg_map = {
        "version": REG_ADDR_VERSION,
        "id": REG_ADDR_DEVICE_ID,
        "zero": REG_ADDR_ZERO_OFFSET,
        "serial": REG_ADDR_SERIAL_NUMBER,
        "angle": None,
    }
    reg = reg_map[args.reg]
    baud = args.baud
    with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
        if reg is None:
            # 读取角度
            if addr == ADDR_BROADCAST:
                print(f"{op_prefix(addr)} [WARN] 广播读取角度会收到多个设备响应，建议指定 --id")
            req = build_frame(addr, FUNC_READ_REQ, b"")
            res = send_and_wait_frames(ser, req, timeout=0.2)
            if not res:
                print(f"{op_prefix(addr)} [ERR] 无响应")
                return 2
            for (raddr, func, data) in res:
                if func == (FUNC_READ_RESP & 0x0F) and len(data) >= 2:
                    ang = ((data[0] << 8) | data[1]) & 0x3FFF
                    print(f"{op_prefix(addr)} [RESP] from id={raddr}: angle={ang}")
            return 0
        else:
            res = read_register(ser, addr, reg, timeout=0.2)
            if not res:
                print(f"{op_prefix(addr)} [ERR] 无响应")
                return 2
            for (raddr, data) in res:
                if reg == REG_ADDR_VERSION and len(data) >= 4:
                    ver = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
                    ver_str = format_version(ver)
                    print(f"{op_prefix(addr)} [RESP] from id={raddr}: version={ver_str} (0x{ver:08X})")
                elif reg == REG_ADDR_DEVICE_ID and len(data) >= 1:
                    print(f"{op_prefix(addr)} [RESP] from id={raddr}: device_id={data[0] & 0x0F}")
                elif reg == REG_ADDR_ZERO_OFFSET and len(data) >= 2:
                    zero = (data[0] << 8) | data[1]
                    print(f"{op_prefix(addr)} [RESP] from id={raddr}: zero_offset={zero}")
                elif reg == REG_ADDR_SERIAL_NUMBER and len(data) >= 6:
                    sn = format_serial_number(data[:6])
                    print(f"{op_prefix(addr)} [RESP] from id={raddr}: serial_number={sn}")
                else:
                    print(f"{op_prefix(addr)} [RESP] from id={raddr}: raw={data.hex()}")
    return 0


def cmd_write(args):
    port = args.port or detect_default_port()
    if not port:
        print("未找到串口，请使用 --port 指定")
        return 1
    addr = ADDR_BROADCAST if args.broadcast else (args.id & 0x0F)
    baud = args.baud
    reg_map = {
        "id": REG_ADDR_DEVICE_ID,
        "zero": REG_ADDR_ZERO_OFFSET,
        "zero_here": REG_ADDR_ZERO_SET_CURRENT,
    }
    reg = reg_map[args.reg]
    if args.reg == "id":
        if args.value is None:
            print("需要 --value <0~15> 设置设备ID")
            return 2
        value = int(args.value, 0) & 0x0F
        payload = bytes([value])
    elif args.reg == "zero":
        if args.value is None:
            print("需要 --value <0~16383> 设置零位")
            return 2
        v = int(args.value, 0) & 0x3FFF
        payload = bytes([(v >> 8) & 0xFF, v & 0xFF])
    elif args.reg == "zero_here":
        payload = b""
    else:
        print("仅支持写入: id / zero / zero_here")
        return 2

    with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
        acks = write_register(ser, addr, reg, payload, timeout=0.3)
        if not acks:
            print(f"{op_prefix(addr)} [ERR] 未收到写入响应(可能设备未在线或总线冲突)")
            return 3
        if addr == ADDR_BROADCAST:
            ids = ", ".join(str(a) for a in sorted(set(acks)))
            print(f"{op_prefix(addr)} [OK] 广播写入已响应的设备: {ids}")
        else:
            print(f"{op_prefix(addr)} [OK] 设备 {addr} 已确认写入")
    return 0


class EncoderTkApp:
    """Tkinter 图形界面。"""

    def __init__(self, root, args):
        import tkinter as tk
        from tkinter import messagebox, ttk

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.root = root
        self.args = args
        self.serial = None
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.poll_stop = threading.Event()
        self.poll_thread: Optional[threading.Thread] = None
        self.last_factory_id: Optional[int] = None
        self.factory_frame = None
        self.factory_target_combo = None
        self.queue: "queue.Queue[Tuple[str, tuple]]" = queue.Queue()

        self.port_var = tk.StringVar(value=args.port or detect_default_port() or "")
        self.baud_var = tk.StringVar(value=str(args.baud or 2_000_000))
        self.current_id_var = tk.StringVar(value="1")
        self.factory_target_var = tk.StringVar(value="1")
        self.scope_var = tk.StringVar(value="target")
        self.read_reg_var = tk.StringVar(value="version")
        self.write_reg_var = tk.StringVar(value="zero_here")
        self.write_value_var = tk.StringVar(value="")
        self.angle_var = tk.StringVar(value="--")
        self.status_var = tk.StringVar(value="未连接")
        self.poll_btn_var = tk.StringVar(value="开始轮询")
        self.last_factory_var = tk.StringVar(value="-")
        self.qc_live_var = tk.StringVar(value="")

        self._build_ui()
        self.root.bind("<Return>", self._on_confirm_key)
        self.root.bind("<KP_Enter>", self._on_confirm_key)
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(60, self._drain_queue)

    def _build_ui(self):
        from tkinter import font as tkfont

        self.root.title("Encoder RS485 Tool (Tkinter)")
        self.root.geometry("1180x520")
        self.root.minsize(980, 500)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=0)

        default_family = tkfont.nametofont("TkDefaultFont").actual("family")
        self.ui_font = tkfont.Font(family=default_family, size=11)
        self.bold_font = tkfont.Font(family=default_family, size=11, weight="bold")
        self.log_font = tkfont.Font(family=default_family, size=11)
        self.step_font = tkfont.Font(family=default_family, size=11, weight="bold")
        self.root.option_add("*Font", self.ui_font)
        style = self.ttk.Style(self.root)
        style.configure(".", font=self.ui_font)
        style.configure("TLabel", font=self.ui_font)
        style.configure("TButton", font=self.ui_font)
        style.configure("TEntry", font=self.ui_font)
        style.configure("TCombobox", font=self.ui_font)
        style.configure("TRadiobutton", font=self.ui_font)
        style.configure("TLabelframe.Label", font=self.bold_font)

        main = self.ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="new")
        main.columnconfigure(0, weight=1)

        conn = self.ttk.LabelFrame(main, text="串口连接")
        conn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        conn.columnconfigure(1, weight=1)

        self.port_combo = self.ttk.Combobox(conn, textvariable=self.port_var, width=22, state="readonly")
        self.port_combo.grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.ttk.Button(conn, text="刷新", command=self.refresh_ports).grid(row=0, column=1, padx=4, pady=4, sticky="w")
        self.ttk.Label(conn, text="波特率").grid(row=0, column=2, padx=(12, 4), pady=4)
        self.baud_entry = self.ttk.Entry(conn, textvariable=self.baud_var, width=12)
        self.baud_entry.grid(row=0, column=3, padx=4, pady=4, sticky="w")
        self.ttk.Button(conn, text="连接", command=self.connect_serial).grid(row=0, column=4, padx=(12, 4), pady=4)
        self.ttk.Button(conn, text="断开", command=self.disconnect_serial).grid(row=0, column=5, padx=4, pady=4)
        self.status_label = self.tk.Label(conn, textvariable=self.status_var, font=self.ui_font, fg="#b91c1c")
        self.status_label.grid(row=0, column=6, padx=(12, 4), pady=4, sticky="w")

        factory = self.ttk.LabelFrame(main, text="工厂流程")
        factory.grid(row=1, column=0, sticky="ew", padx=2, pady=4)
        self.factory_frame = factory
        factory.columnconfigure(6, weight=1)
        self.ttk.Label(factory, text="工厂目标ID").grid(row=0, column=0, padx=4, pady=4)
        self.factory_target_combo = self.ttk.Combobox(
            factory,
            textvariable=self.factory_target_var,
            values=[str(i) for i in range(1, 8)],
            width=6,
            state="readonly",
        )
        self.factory_target_combo.grid(row=0, column=1, padx=4, pady=4, sticky="w")
        self.factory_target_combo.bind("<Return>", self._on_confirm_key, add="+")
        self.factory_target_combo.bind("<KP_Enter>", self._on_confirm_key, add="+")
        self.ttk.Label(factory, text="上次目标ID").grid(row=0, column=2, padx=(12, 4), pady=4)
        self.ttk.Label(factory, textvariable=self.last_factory_var, width=8, relief="sunken").grid(row=0, column=3, padx=4, pady=4)
        self.ttk.Button(factory, text="执行工厂流程", command=self.run_factory_flow).grid(row=0, column=4, padx=(12, 4), pady=4)
        self.ttk.Label(factory, text="选好目标后按 Enter 执行；流程会广播检查版本 -> 广播设ID -> 广播置零 -> 目标ID质检").grid(row=0, column=5, padx=12, pady=4, sticky="w")

        target = self.ttk.LabelFrame(main, text="目标 / 广播")
        target.grid(row=2, column=0, sticky="ew", padx=2, pady=6)
        for col in range(6):
            target.columnconfigure(col, weight=0)
        target.columnconfigure(5, weight=1)
        self.ttk.Label(target, text="当前ID").grid(row=0, column=0, padx=4, pady=4)
        self.current_id_entry = self.ttk.Entry(target, textvariable=self.current_id_var, width=8)
        self.current_id_entry.grid(row=0, column=1, padx=4, pady=4, sticky="w")
        self.ttk.Label(target, text="范围").grid(row=0, column=2, padx=(12, 4), pady=4)
        self.ttk.Radiobutton(target, text="定向", value="target", variable=self.scope_var).grid(row=0, column=3, padx=4, pady=4)
        self.ttk.Radiobutton(target, text="广播", value="broadcast", variable=self.scope_var).grid(row=0, column=4, padx=4, pady=4)
        self.ttk.Label(target, text="当前角度").grid(row=0, column=5, padx=(12, 4), pady=4, sticky="e")
        self.angle_label = self.ttk.Label(target, textvariable=self.angle_var, width=14, relief="sunken")
        self.angle_label.grid(row=0, column=6, padx=4, pady=4, sticky="w")

        ops = self.ttk.LabelFrame(main, text="操作")
        ops.grid(row=3, column=0, sticky="ew", padx=2, pady=4)
        ops.columnconfigure(9, weight=1)

        self.ttk.Label(ops, text="寄存器").grid(row=0, column=0, padx=4, pady=4)
        self.read_reg_combo = self.ttk.Combobox(
            ops,
            textvariable=self.read_reg_var,
            values=["version", "id", "zero", "serial", "angle"],
            width=10,
            state="readonly",
        )
        self.read_reg_combo.grid(row=0, column=1, padx=4, pady=4, sticky="w")
        self.ttk.Button(ops, text="读取角度", command=self.read_angle).grid(row=0, column=2, padx=(12, 4), pady=4)
        self.ttk.Button(ops, text="读取寄存器", command=self.read_register).grid(row=0, column=3, padx=4, pady=4)
        self.ttk.Label(ops, text="写寄存器").grid(row=0, column=4, padx=(12, 4), pady=4)
        self.write_reg_combo = self.ttk.Combobox(
            ops,
            textvariable=self.write_reg_var,
            values=["id", "zero", "zero_here"],
            width=10,
            state="readonly",
        )
        self.write_reg_combo.grid(row=0, column=5, padx=4, pady=4, sticky="w")
        self.ttk.Entry(ops, textvariable=self.write_value_var, width=12).grid(row=0, column=6, padx=4, pady=4, sticky="w")
        self.ttk.Button(ops, text="写寄存器", command=self.write_register).grid(row=0, column=7, padx=4, pady=4)
        self.poll_btn = self.ttk.Button(ops, textvariable=self.poll_btn_var, command=self.toggle_polling)
        self.poll_btn.grid(row=0, column=8, padx=(12, 4), pady=4)
        self.ttk.Button(ops, text="清空日志", command=self.clear_log).grid(row=0, column=9, padx=4, pady=4, sticky="w")

        logs = self.ttk.LabelFrame(main, text="日志")
        logs.grid(row=4, column=0, sticky="ew", padx=2, pady=4)
        logs.columnconfigure(0, weight=1)
        self.qc_live_label = self.tk.Label(logs, textvariable=self.qc_live_var, anchor="w", justify="left", fg="#2563eb", font=self.step_font)
        self.qc_live_label.grid(row=0, column=0, columnspan=2, sticky="ew", padx=2, pady=(2, 4))
        self.log_text = self.tk.Text(logs, wrap="word", height=9, font=self.log_font)
        self.log_text.grid(row=1, column=0, sticky="ew")
        scroll = self.ttk.Scrollbar(logs, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=1, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.tag_configure("info", foreground="#1f2937")
        self.log_text.tag_configure("ok", foreground="#047857")
        self.log_text.tag_configure("warn", foreground="#b45309")
        self.log_text.tag_configure("error", foreground="#b91c1c")
        self.log_text.tag_configure("step", foreground="#2563eb", font=self.step_font)

        self._log("info", "Tkinter GUI 已启动。")
        self.root.update_idletasks()
        req_w = max(self.root.winfo_reqwidth(), 980)
        req_h = max(self.root.winfo_reqheight(), 500)
        self.root.geometry(f"{req_w}x{req_h}")

    def refresh_ports(self):
        ports = list_all_ports()
        self.port_combo["values"] = ports
        if ports and (self.port_var.get() not in ports):
            default = self.args.port or detect_default_port() or ports[0]
            if default in ports:
                self.port_var.set(default)
            else:
                self.port_var.set(ports[0])
        elif not self.port_var.get() and ports:
            self.port_var.set(ports[0])
        self._log("info", f"已刷新串口列表：{', '.join(ports) if ports else '(无)'}")

    def _parse_int(self, text: str, name: str) -> int:
        try:
            return int(text.strip(), 0)
        except Exception as e:
            raise ValueError(f"{name} 无效: {e}")

    def _selected_addr(self) -> int:
        if self.scope_var.get() == "broadcast":
            return ADDR_BROADCAST
        return self._parse_int(self.current_id_var.get(), "当前ID") & 0x0F

    def _selected_factory_target(self) -> int:
        text = self.factory_target_var.get().strip()
        if not text:
            if self.last_factory_id is None:
                raise ValueError("未填写工厂目标ID，且没有上次ID可用")
            return int(self.last_factory_id) & 0x0F
        value = self._parse_int(text, "工厂目标ID")
        if not 1 <= value <= 7:
            raise ValueError("工厂目标ID 仅支持 1~7")
        return value & 0x0F

    def _reg_to_code(self, reg_name: str) -> Optional[int]:
        mapping = {
            "version": REG_ADDR_VERSION,
            "id": REG_ADDR_DEVICE_ID,
            "zero": REG_ADDR_ZERO_OFFSET,
            "serial": REG_ADDR_SERIAL_NUMBER,
            "angle": None,
        }
        if reg_name not in mapping:
            raise ValueError("寄存器名称无效")
        return mapping[reg_name]

    def _log(self, level: str, message: str):
        self.queue.put(("log", (level, message)))

    def _set_var(self, name: str, value: str):
        self.queue.put(("setvar", (name, value)))

    def _set_status(self, text: str):
        self.queue.put(("status", (text,)))

    def _set_angle(self, text: str):
        self.queue.put(("angle", (text,)))

    def _set_qc_live(self, text: str):
        self.queue.put(("qc_live", (text,)))

    def _set_polling_ui(self, active: bool):
        self.queue.put(("polling", (active,)))

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _append_log(self, level: str, message: str):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n", level if level in ("info", "ok", "warn", "error", "step") else "info")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _status_color(self, text: str) -> str:
        if text.startswith("已连接"):
            return "#047857"
        if text.startswith("未连接"):
            return "#b91c1c"
        if "失败" in text or "断开" in text:
            return "#b91c1c"
        if "连接中" in text:
            return "#b45309"
        return "#1f2937"

    def _apply_status(self, text: str):
        self.status_var.set(text)
        self.status_label.configure(fg=self._status_color(text))

    def _on_confirm_key(self, event=None):
        widget = getattr(event, "widget", None) or self.root.focus_get()
        if widget is None:
            return None
        focus_name = str(widget)
        factory_name = str(self.factory_frame) if self.factory_frame is not None else ""
        if widget == self.factory_target_combo or focus_name.startswith(factory_name):
            self.run_factory_flow()
            return "break"
        return None

    def _drain_queue(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "log":
                    level, message = payload
                    self._append_log(level, message)
                elif kind == "setvar":
                    name, value = payload
                    if name == "current_id":
                        self.current_id_var.set(value)
                    elif name == "factory_target":
                        self.factory_target_var.set(value)
                    elif name == "last_factory":
                        self.last_factory_var.set(value)
                elif kind == "status":
                    (text,) = payload
                    self._apply_status(text)
                elif kind == "angle":
                    (text,) = payload
                    self.angle_var.set(text)
                elif kind == "qc_live":
                    (text,) = payload
                    self.qc_live_var.set(text)
                elif kind == "polling":
                    (active,) = payload
                    self.poll_btn_var.set("停止轮询" if active else "开始轮询")
        except queue.Empty:
            pass
        if not self.stop_event.is_set():
            self.root.after(60, self._drain_queue)

    def _open_serial_locked(self):
        port = self.port_var.get().strip()
        if not port:
            raise ValueError("请选择串口")
        baud = self._parse_int(self.baud_var.get(), "波特率")
        if baud <= 0:
            raise ValueError("波特率必须大于 0")
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=0)
        self._set_status(f"已连接 {port} @ {baud}")
        self._log("ok", f"已连接串口 {port} @ {baud}")

    def connect_serial(self):
        try:
            with self.serial_lock:
                self._open_serial_locked()
        except Exception as e:
            self._set_status("连接失败")
            self._log("error", f"打开串口失败: {e}")
            self.messagebox.showerror("连接失败", str(e))

    def disconnect_serial(self):
        self.stop_polling()
        with self.serial_lock:
            if self.serial is not None:
                try:
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None
        self._set_status("已断开")
        self._log("info", "串口已断开")

    def _worker(self, title: str, fn):
        def runner():
            try:
                fn()
            except Exception as e:
                self._log("error", f"{title} 失败: {e}")
        threading.Thread(target=runner, daemon=True).start()

    def _format_read_result(self, reg_name: str, addr: int, data: bytes, raddr: int) -> str:
        if reg_name == "version" and len(data) >= 4:
            ver = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
            return f"ID={raddr} 版本={format_version(ver)} raw=0x{data[:4].hex().upper()}"
        if reg_name == "id" and len(data) >= 1:
            return f"ID={raddr} 设备ID={data[0]} raw=0x{data[:1].hex().upper()}"
        if reg_name == "zero" and len(data) >= 2:
            zero = (data[0] << 8) | data[1]
            return f"ID={raddr} 零位={zero} raw=0x{data[:2].hex().upper()}"
        if reg_name == "serial" and len(data) >= 6:
            return f"ID={raddr} 序列号={format_serial_number(data[:6])}"
        if reg_name == "angle" and len(data) >= 2:
            ang = ((data[0] << 8) | data[1]) & 0x3FFF
            return f"ID={raddr} 角度={ang}"
        return f"ID={raddr} 数据={data.hex().upper()}"

    def read_angle(self):
        try:
            addr = self._selected_addr()
        except Exception as e:
            self._log("error", str(e))
            return

        def job():
            with self.serial_lock:
                if self.serial is None:
                    raise RuntimeError("未连接串口")
                result = read_angle(self.serial, addr, timeout=0.08)
            if result is None:
                self._log("warn", "读取角度无响应")
                return
            rid, ang = result
            self._set_angle(str(ang))
            self._log("ok", f"角度读取成功：ID={rid} angle={ang}")
        self._worker("读取角度", job)

    def read_register(self):
        try:
            reg_name = self.read_reg_var.get().strip().lower()
            addr = self._selected_addr()
        except Exception as e:
            self._log("error", str(e))
            return

        def job():
            reg = self._reg_to_code(reg_name)
            if reg is None:
                with self.serial_lock:
                    if self.serial is None:
                        raise RuntimeError("未连接串口")
                    result = read_angle(self.serial, addr, timeout=0.08)
                if result is None:
                    self._log("warn", "读取角度无响应")
                else:
                    rid, ang = result
                    self._set_angle(str(ang))
                    self._log("ok", f"角度读取成功：ID={rid} angle={ang}")
                return
            with self.serial_lock:
                if self.serial is None:
                    raise RuntimeError("未连接串口")
                results = read_register(self.serial, addr, reg, timeout=0.2)
            if not results:
                self._log("warn", "读取寄存器无响应")
                return
            for raddr, data in results:
                self._log("ok", self._format_read_result(reg_name, addr, data, raddr))
        self._worker("读取寄存器", job)

    def write_register(self):
        try:
            reg_name = self.write_reg_var.get().strip().lower()
            addr = self._selected_addr()
            raw_value = self.write_value_var.get()
        except Exception as e:
            self._log("error", str(e))
            return

        def job():
            if reg_name == "id":
                value = self._parse_int(raw_value, "设备ID") & 0x0F
                payload = bytes([value])
                reg = REG_ADDR_DEVICE_ID
            elif reg_name == "zero":
                value = self._parse_int(raw_value, "零位") & 0x3FFF
                payload = bytes([(value >> 8) & 0xFF, value & 0xFF])
                reg = REG_ADDR_ZERO_OFFSET
            elif reg_name == "zero_here":
                payload = b""
                reg = REG_ADDR_ZERO_SET_CURRENT
            else:
                raise ValueError("仅支持写入 id / zero / zero_here")

            with self.serial_lock:
                if self.serial is None:
                    raise RuntimeError("未连接串口")
                acks = write_register(self.serial, addr, reg, payload, timeout=0.3)
            if not acks:
                self._log("error", "未收到写入响应")
                return
            if addr == ADDR_BROADCAST:
                ids = ", ".join(str(a) for a in sorted(set(acks)))
                self._log("ok", f"广播写入已响应的设备: {ids}")
            else:
                self._log("ok", f"设备 {addr} 已确认写入")
        self._worker("写寄存器", job)

    def _poll_loop(self, addr: int):
        self._set_status("轮询中")
        self._set_polling_ui(True)
        self._log("step", "轮询已启动")
        try:
            while not self.poll_stop.is_set() and not self.stop_event.is_set():
                with self.serial_lock:
                    if self.serial is None:
                        self._log("error", "串口已断开，停止轮询")
                        break
                    result = read_angle(self.serial, addr, timeout=0.05)
                if result is not None:
                    rid, ang = result
                    self._set_angle(str(ang))
                    self.queue.put(("log", ("info", f"轮询：ID={rid} angle={ang}")))
                time.sleep(0.05)
        finally:
            self._set_polling_ui(False)
            if self.serial is not None:
                self._set_status("已连接")
            self._log("step", "轮询已停止")

    def start_polling(self):
        if self.poll_thread and self.poll_thread.is_alive():
            return
        try:
            addr = self._selected_addr()
            with self.serial_lock:
                if self.serial is None:
                    raise RuntimeError("请先连接串口")
        except Exception as e:
            self._log("error", str(e))
            return
        self.poll_stop.clear()
        self.poll_thread = threading.Thread(target=self._poll_loop, args=(addr,), daemon=True)
        self.poll_thread.start()

    def stop_polling(self):
        self.poll_stop.set()
        if self.poll_thread and self.poll_thread.is_alive():
            self.poll_thread.join(timeout=1.0)
        self.poll_thread = None

    def toggle_polling(self):
        if self.poll_thread and self.poll_thread.is_alive():
            self.stop_polling()
        else:
            self.start_polling()

    def run_factory_flow(self):
        try:
            target_id = self._selected_factory_target()
        except Exception as e:
            self._log("error", str(e))
            return
        self.last_factory_id = target_id
        self.last_factory_var.set(str(target_id))

        def job():
            self.stop_polling()
            self._set_var("last_factory", str(target_id))
            self._log("step", f"工厂流程开始，目标ID={target_id}")

            with self.serial_lock:
                if self.serial is None:
                    raise RuntimeError("未连接串口")
                ver_results = read_register(self.serial, ADDR_BROADCAST, REG_ADDR_VERSION, timeout=0.3)
            if not ver_results:
                self._log("warn", "广播版本检查无响应，继续执行")
            else:
                version_ok = True
                for raddr, data in ver_results:
                    if len(data) >= 4:
                        ver = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
                        self._log("info", f"设备ID={raddr} 版本={format_version(ver)}")
                        if ver < FIRMWARE_VERSION_LATEST:
                            version_ok = False
                    else:
                        self._log("warn", f"设备ID={raddr} 版本数据不足")
                if not version_ok:
                    self._log("warn", "检测到旧版本固件，仍继续执行")

            with self.serial_lock:
                if self.serial is None:
                    raise RuntimeError("未连接串口")
                acks = write_register(self.serial, ADDR_BROADCAST, REG_ADDR_DEVICE_ID, bytes([target_id]), timeout=0.3)
            if not acks:
                raise RuntimeError("广播设ID无响应")
            self._log("ok", f"广播设ID完成，响应设备: {', '.join(str(a) for a in sorted(set(acks)))}")

            time.sleep(0.2)
            with self.serial_lock:
                if self.serial is None:
                    raise RuntimeError("未连接串口")
                acks2 = write_register(self.serial, ADDR_BROADCAST, REG_ADDR_ZERO_SET_CURRENT, b"", timeout=0.3)
            if not acks2:
                raise RuntimeError("广播置零无响应")
            self._log("ok", f"广播置零完成，响应设备: {', '.join(str(a) for a in sorted(set(acks2)))}")

            self._set_var("current_id", str(target_id))
            self.last_factory_id = target_id
            self._set_status(f"工厂流程进行中，ID={target_id}")

            ok_cnt = 0
            deadline = time.time() + 2.0
            self._set_qc_live(f"质检：ID={target_id} angle=--")
            while time.time() < deadline and not self.stop_event.is_set():
                with self.serial_lock:
                    if self.serial is None:
                        break
                    result = read_angle(self.serial, target_id, timeout=0.05)
                if result is not None:
                    _, ang = result
                    ok_cnt += 1
                    self._set_angle(str(ang))
                    self._set_qc_live(f"质检：ID={target_id} angle={ang}")
                time.sleep(0.05)
            self._set_qc_live("")

            if ok_cnt > 0:
                self._log("ok", "工厂流程完成，质检通过")
            else:
                self._log("warn", "工厂流程完成，但质检期间未读到角度")
            self._set_status(f"工厂流程完成，目标ID={target_id}")

        self._worker("工厂流程", job)

    def close(self):
        self.stop_event.set()
        self.stop_polling()
        self.disconnect_serial()
        self.root.destroy()


def cmd_gui(args):
    try:
        import tkinter as tk
    except Exception as e:
        print(f"[ERROR] 无法导入 Tkinter: {e}")
        return 1
    try:
        root = tk.Tk()
    except Exception as e:
        print(f"[ERROR] 无法启动图形界面: {e}")
        return 1
    app = EncoderTkApp(root, args)
    root.mainloop()
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Encoder RS485 通信工具 (usb2485)")
    p.add_argument("--port", help="串口设备路径，如 /dev/tty.usbserial-xxxx")
    p.add_argument("--baud", type=int, default=2000000, help="波特率，默认 2000000")

    sub = p.add_subparsers(dest="cmd")

    # 公共父解析器，允许在子命令后使用 --port/--baud
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", help="串口设备路径，如 /dev/tty.usbserial-xxxx")
    common.add_argument("--baud", type=int, help="波特率(覆盖全局)")

    sp_poll = sub.add_parser("poll", parents=[common], help="根据设备ID持续读取角度")
    sp_poll.add_argument("--id", type=int, required=True, help="设备ID(1~15)")
    sp_poll.add_argument("--interval", type=float, default=0.05, help="读取间隔秒，默认0.05s")
    sp_poll.set_defaults(func=cmd_poll)

    sp_read = sub.add_parser("read", parents=[common], help="读取寄存器或角度")
    sp_read.add_argument("--id", type=int, help="设备ID(1~15)，若不指定且 --broadcast 则为广播")
    sp_read.add_argument("--broadcast", action="store_true", help="使用广播地址0读取")
    sp_read.add_argument("--reg", choices=["version", "id", "zero", "serial", "angle"], required=True, help="寄存器或角度")
    sp_read.set_defaults(func=cmd_read)

    sp_write = sub.add_parser("write", parents=[common], help="写寄存器（支持广播）")
    sp_write.add_argument("--id", type=int, help="设备ID(1~15)，与 --broadcast 二选一")
    sp_write.add_argument("--broadcast", action="store_true", help="使用广播写入")
    sp_write.add_argument("--reg", choices=["id", "zero", "zero_here"], required=True, help="寄存器: id/zero/zero_here")
    sp_write.add_argument("--value", help="写入值，id:0~15，zero:0~16383；zero_here不需要")
    sp_write.set_defaults(func=cmd_write)

    sp_ui = sub.add_parser("ui", parents=[common], help="图形化界面 (Tkinter)")
    sp_ui.set_defaults(func=cmd_gui)

    sp_gui = sub.add_parser("gui", parents=[common], help="图形化界面 (Tkinter)")
    sp_gui.set_defaults(func=cmd_gui)

    sp_textui = sub.add_parser("textui", parents=[common], help="旧版命令行交互界面（保留）")
    sp_textui.set_defaults(func=cmd_ui)

    return p


def main(argv: List[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not hasattr(args, "func"):
        # 默认进入旧版命令行交互模式（textui）。
        args.func = cmd_ui
    try:
        return args.func(args)
    except KeyboardInterrupt:
        print("\n[INFO] 退出")
        return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


