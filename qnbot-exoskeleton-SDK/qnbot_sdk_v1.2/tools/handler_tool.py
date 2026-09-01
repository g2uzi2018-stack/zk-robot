#!/usr/bin/env python3
import argparse
import sys
import time
from typing import List, Optional, Tuple
import select
import tty
import termios
import subprocess

try:
	import serial  # pyserial
	import serial.tools.list_ports as list_ports
except Exception as e:
	print("[ERROR] 需要安装 pyserial: pip install pyserial", file=sys.stderr)
	raise

# 协议常量
FRAME_HEADER = 0x55
FRAME_TAIL = 0xAA
FRAME_MAX_SIZE = 16

# 功能码
FUNC_READ_REQ = 0x01
FUNC_READ_RESP = 0x01 | 0x04  # 0x05
FUNC_WRITE_REQ = 0x02
FUNC_WRITE_RESP = 0x02 | 0x04  # 0x06

# 寄存器地址（与 firmware 中 proto_comm.h 保持一致）
REG_ADDR_VERSION = 0x00
REG_ADDR_DEVICE_ID = 0x01
# 动作写寄存器（将“当前读数”持久化）
REG_ADDR_SET_JX_CENTER_CURRENT = 0x10
REG_ADDR_SET_JX_MAX_CURRENT = 0x11
REG_ADDR_SET_JX_MIN_CURRENT = 0x12
REG_ADDR_SET_JY_CENTER_CURRENT = 0x13
REG_ADDR_SET_JY_MAX_CURRENT = 0x14
REG_ADDR_SET_JY_MIN_CURRENT = 0x15
REG_ADDR_SET_TRIG_START_CURRENT = 0x16
REG_ADDR_SET_TRIG_MAX_CURRENT = 0x17
# 只读标定查询
REG_ADDR_READ_JX_CENTER = 0x20
REG_ADDR_READ_JX_MAX = 0x21
REG_ADDR_READ_JX_MIN = 0x22
REG_ADDR_READ_JY_CENTER = 0x23
REG_ADDR_READ_JY_MAX = 0x24
REG_ADDR_READ_JY_MIN = 0x25
REG_ADDR_READ_TRIG_START = 0x26
REG_ADDR_READ_TRIG_MAX = 0x27
# 校准输出开关（读写1B：0/1）
REG_ADDR_CALIB_OUTPUT_ENABLE = 0x28

# 广播地址
ADDR_BROADCAST = 0x00


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


def beep(times: int = 1):
	"""多平台提示音：macOS 优先 osascript beep；Windows 用 winsound；其他用终端铃声"""
	times = max(1, int(times))
	for _ in range(times):
		try:
			if sys.platform.startswith("darwin"):
				subprocess.run(["osascript", "-e", "beep"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
			elif sys.platform.startswith("win"):
				try:
					import winsound
					winsound.Beep(800, 150)
				except Exception:
					sys.stdout.write("\a"); sys.stdout.flush()
			else:
				sys.stdout.write("\a"); sys.stdout.flush()
		except Exception:
			try:
				sys.stdout.write("\a"); sys.stdout.flush()
			except Exception:
				pass
		time.sleep(0.05)


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
	frames = []
	i = 0
	while i + 4 <= len(buf):
		if buf[i] != FRAME_HEADER:
			i += 1
			continue
		end_limit = min(i + FRAME_MAX_SIZE, len(buf))
		tail_pos = -1
		for j in range(i + 4, end_limit + 1):
			if j <= len(buf) and j - i >= 4:
				if buf[j - 1] == FRAME_TAIL:
					tail_pos = j - 1
					break
		if tail_pos == -1:
			break
		frame = bytes(buf[i: tail_pos + 1])
		addr_func = frame[1]
		data = frame[2:-2]
		xor_calc = addr_func
		for b in data:
			xor_calc ^= b
		xor_rx = frame[-2]
		if xor_calc == xor_rx:
			frames.append(frame)
			del buf[: tail_pos + 1]
			i = 0
		else:
			i += 1
			del buf[:i]
			i = 0
	return frames


def decode_frame(frame: bytes) -> Tuple[int, int, bytes]:
	if len(frame) < 4 or frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
		raise ValueError("invalid frame")
	addr_func = frame[1]
	addr = (addr_func >> 4) & 0x0F
	func = addr_func & 0x0F
	data = frame[2:-2]
	return addr, func, data


def send_and_wait_frames(ser: serial.Serial, req: bytes, timeout: float = 0.15) -> List[Tuple[int, int, bytes]]:
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


def read_register(ser: serial.Serial, addr: int, reg: int, timeout: float = 0.2) -> List[Tuple[int, bytes]]:
	req = build_frame(addr, FUNC_READ_REQ, bytes([reg]))
	resps = send_and_wait_frames(ser, req, timeout)
	out: List[Tuple[int, bytes]] = []
	for (raddr, func, data) in resps:
		if func == (FUNC_READ_RESP & 0x0F):
			out.append((raddr, data))
	return out


def write_register(ser: serial.Serial, addr: int, reg: int, payload: bytes = b"", timeout: float = 0.2) -> List[int]:
	req = build_frame(addr, FUNC_WRITE_REQ, bytes([reg]) + payload)
	resps = send_and_wait_frames(ser, req, timeout)
	acks: List[int] = []
	for (raddr, func, data) in resps:
		if func == (FUNC_WRITE_RESP & 0x0F):
			acks.append(raddr)
	return acks


def read_state(ser: serial.Serial, addr: int, timeout: float = 0.15) -> Optional[Tuple[int, int, int, int, int]]:
	"""
	读取默认状态帧(无寄存器)：返回 (resp_id, x, y, btn, trig)
	"""
	req = build_frame(addr, FUNC_READ_REQ, b"")
	resps = send_and_wait_frames(ser, req, timeout)
	for (raddr, func, data) in resps:
		if func == (FUNC_READ_RESP & 0x0F) and len(data) >= 7:
			x = ((data[0] << 8) | data[1]) & 0xFFFF
			y = ((data[2] << 8) | data[3]) & 0xFFFF
			btn = data[4] & 0xFF
			trig = ((data[5] << 8) | data[6]) & 0xFFFF
			return (raddr, x, y, btn, trig)
	return None


def ensure_calib_output_off(ser: serial.Serial, addr: int) -> bool:
	res = read_register(ser, addr, REG_ADDR_CALIB_OUTPUT_ENABLE, timeout=0.2)
	val = None
	for (_, data) in res:
		if len(data) >= 1:
			val = data[0] & 0x01
			break
	if val is None:
		print("[WARN] 未能读取校准输出寄存器，继续", flush=True)
		return False
	if val != 0:
		acks = write_register(ser, addr, REG_ADDR_CALIB_OUTPUT_ENABLE, bytes([0]))
		ok = (addr == ADDR_BROADCAST and len(acks) > 0) or (addr in acks)
		if not ok:
			print("[ERR] 关闭校准输出失败", flush=True)
			return False
	return True


def prompt_live_and_commit(ser: serial.Serial, addr: int, prompt: str, commit_reg: int):
	print(prompt + "（按回车提交，Ctrl+C 取消）")
	old_term = None
	try:
		old_term = termios.tcgetattr(sys.stdin.fileno())
		tty.setcbreak(sys.stdin.fileno())
	except Exception:
		old_term = None
	try:
		while True:
			st = read_state(ser, addr, timeout=0.08)
			if st is not None:
				_, x, y, btn, trig = st
				print(f"\rX={x:4d}  Y={y:4d}  TRIG={trig:4d}  BTN=0x{btn:02X}      ", end="", flush=True)
			else:
				print("\r(无数据)                                           ", end="", flush=True)
			r, _, _ = select.select([sys.stdin], [], [], 0.02)
			if r:
				ch = sys.stdin.read(1)
				if ch == '\n' or ch == '\r':
					break
			time.sleep(0.02)
	finally:
		print()
		if old_term is not None:
			try:
				termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
			except Exception:
				pass
	acks = write_register(ser, addr, commit_reg, b"", timeout=0.3)
	ok = (addr == ADDR_BROADCAST and len(acks) > 0) or (addr in acks)
	print("[OK] 已提交" if ok else "[ERR] 提交失败")
	return ok


def qc_check_value(name: str, value: int, target: int, tol: int) -> Tuple[bool, str]:
	ok = abs(int(value) - int(target)) <= tol
	msg = f"{name}: {value} vs {target} (±{tol}) -> {'OK' if ok else 'NG'}"
	return ok, msg


def run_factory(ser: serial.Serial, addr: int):
	print("[STEP 1] 确保“校准输出”为0（关闭映射）")
	ensure_calib_output_off(ser, addr)
	beep()

	print("[STEP 2] 校准中心/起始（保持静止）")
	prompt_live_and_commit(ser, addr, "保持摇杆/板机静止，校准 X 中心", REG_ADDR_SET_JX_CENTER_CURRENT)
	prompt_live_and_commit(ser, addr, "保持摇杆/板机静止，校准 Y 中心", REG_ADDR_SET_JY_CENTER_CURRENT)
	prompt_live_and_commit(ser, addr, "保持摇杆/板机静止，校准 TRIGGER 起始", REG_ADDR_SET_TRIG_START_CURRENT)
	beep()

	print("[STEP 3] 校准极值（按提示移动/捏住）")
	print("提示：X 最大（往左），准备就绪后回车")
	prompt_live_and_commit(ser, addr, "请将摇杆推到 X 最大（往左）", REG_ADDR_SET_JX_MAX_CURRENT)
	print("提示：X 最小（往右），准备就绪后回车")
	prompt_live_and_commit(ser, addr, "请将摇杆推到 X 最小（往右）", REG_ADDR_SET_JX_MIN_CURRENT)
	print("提示：Y 最大（往上），准备就绪后回车")
	prompt_live_and_commit(ser, addr, "请将摇杆推到 Y 最大（往上）", REG_ADDR_SET_JY_MAX_CURRENT)
	print("提示：Y 最小（往下），准备就绪后回车")
	prompt_live_and_commit(ser, addr, "请将摇杆推到 Y 最小（往下）", REG_ADDR_SET_JY_MIN_CURRENT)
	print("提示：板机最大（捏到底），准备就绪后回车")
	prompt_live_and_commit(ser, addr, "请捏住板机至最大", REG_ADDR_SET_TRIG_MAX_CURRENT)
	beep()

	print("[STEP 4] 质检校验：开启映射输出")
	acks = write_register(ser, addr, REG_ADDR_CALIB_OUTPUT_ENABLE, bytes([1]), timeout=0.3)
	ok = (addr == ADDR_BROADCAST and len(acks) > 0) or (addr in acks)
	if not ok:
		print("[ERR] 打开映射输出失败，终止质检")
		return 2
	beep()

	print("请按以下姿态依次进行，并在每个动作稳定后按回车采样：")
	def wait_enter(label: str) -> Optional[Tuple[int,int,int,int,int]]:
		print(f" - {label}：稳定后按回车采样")
		old_term = None
		try:
			old_term = termios.tcgetattr(sys.stdin.fileno())
			tty.setcbreak(sys.stdin.fileno())
		except Exception:
			old_term = None
		sample = None
		try:
			while True:
				st = read_state(ser, addr, timeout=0.08)
				if st is not None:
					_, x, y, btn, trig = st
					print(f"\rX={x:4d}  Y={y:4d}  TRIG={trig:4d}  BTN=0x{btn:02X}      ", end="", flush=True)
					sample = st
				else:
					print("\r(无数据)                                           ", end="", flush=True)
				r, _, _ = select.select([sys.stdin], [], [], 0.02)
				if r:
					ch = sys.stdin.read(1)
					if ch == '\n' or ch == '\r':
						break
				time.sleep(0.02)
		finally:
			print()
			if old_term is not None:
				try:
					termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
				except Exception:
					pass
		return sample

	# 目标值与容差
	TOL = 120
	targets = [
		("静止(中心/起始)", 2048, 2048, 512),
		("板机最大", None, None, 3584),
	]
	results = []
	for label, tx, ty, tt in targets:
		s = wait_enter(label)
		if s is None:
			print("[ERR] 未采集到数据，终止")
			return 3
		_, x, y, _, trig = s
		local_ok = True
		if tx is not None:
			ok_x, msg_x = qc_check_value(f"{label}/X", x, tx, TOL)
			results.append(msg_x); local_ok = local_ok and ok_x
		if ty is not None:
			ok_y, msg_y = qc_check_value(f"{label}/Y", y, ty, TOL)
			results.append(msg_y); local_ok = local_ok and ok_y
		if tt is not None:
			ok_t, msg_t = qc_check_value(f"{label}/TRIG", trig, tt, TOL)
			results.append(msg_t); local_ok = local_ok and ok_t
		if not local_ok:
			results.append(f"{label}: NG")

	# 追加：旋转6秒极值范围质检（不替代姿态质检）
	print("旋转质检：请绕圈持续旋转摇杆约6秒，检测输出极值范围。")
	t0 = time.time()
	qc_min_x, qc_max_x = 0xFFFF, 0
	qc_min_y, qc_max_y = 0xFFFF, 0
	while time.time() - t0 < 6.0:
		st = read_state(ser, addr, timeout=0.08)
		if st is not None:
			_, x, y, _, _ = st
			if x < qc_min_x: qc_min_x = x
			if x > qc_max_x: qc_max_x = x
			if y < qc_min_y: qc_min_y = y
			if y > qc_max_y: qc_max_y = y
		time.sleep(0.01)
	# 追加极值质检结果
	_, msg1 = qc_check_value("X 左最大/X", qc_max_x, 3848, TOL); results.append(msg1)
	_, msg2 = qc_check_value("X 右最小/X", qc_min_x, 248, TOL);  results.append(msg2)
	_, msg3 = qc_check_value("Y 上最大/Y", qc_max_y, 3848, TOL); results.append(msg3)
	_, msg4 = qc_check_value("Y 下最小/Y", qc_min_y, 248, TOL);  results.append(msg4)

	print("\n[STEP 5] 质检结果")
	all_ok = True
	for line in results:
		print("  - " + line)
		if line.endswith("NG"):
			all_ok = False
	print("\n最终结果: " + ("PASS" if all_ok else "FAIL"))
	beep(2)
	return 0 if all_ok else 4


def run_calib_joystick(ser: serial.Serial, addr: int):
	print("[Joystick] 先关闭映射输出")
	ensure_calib_output_off(ser, addr)
	prompt_live_and_commit(ser, addr, "保持静止 -> 校准 X 中心", REG_ADDR_SET_JX_CENTER_CURRENT)
	prompt_live_and_commit(ser, addr, "保持静止 -> 校准 Y 中心", REG_ADDR_SET_JY_CENTER_CURRENT)
	beep()
	prompt_live_and_commit(ser, addr, "将 X 推到最大（左）", REG_ADDR_SET_JX_MAX_CURRENT)
	prompt_live_and_commit(ser, addr, "将 X 推到最小（右）", REG_ADDR_SET_JX_MIN_CURRENT)
	prompt_live_and_commit(ser, addr, "将 Y 推到最大（上）", REG_ADDR_SET_JY_MAX_CURRENT)
	prompt_live_and_commit(ser, addr, "将 Y 推到最小（下）", REG_ADDR_SET_JY_MIN_CURRENT)
	print("[Joystick] 完成")
	beep(2)
	return 0


def run_calib_trigger(ser: serial.Serial, addr: int):
	print("[Trigger] 先关闭映射输出")
	ensure_calib_output_off(ser, addr)
	prompt_live_and_commit(ser, addr, "保持静止 -> 校准 TRIGGER 起始", REG_ADDR_SET_TRIG_START_CURRENT)
	prompt_live_and_commit(ser, addr, "捏住板机 -> 校准 TRIGGER 最大", REG_ADDR_SET_TRIG_MAX_CURRENT)
	print("[Trigger] 完成")
	beep(2)
	return 0


def run_smart_calib(ser: serial.Serial, addr: int):
	print("[Smart] 确保“校准输出”为0（关闭映射）")
	ensure_calib_output_off(ser, addr)

	# 0) 工具：等待回车后采样 duration 秒的均值
	def sample_mean(duration_s: float) -> Optional[Tuple[int,int,int,int,int]]:
		print("  按回车开始采样...")
		try:
			old_term = termios.tcgetattr(sys.stdin.fileno())
			tty.setcbreak(sys.stdin.fileno())
		except Exception:
			old_term = None
		try:
			# 等待回车，同时实时显示当前值
			while True:
				r, _, _ = select.select([sys.stdin], [], [], 0.05)
				if r:
					ch = sys.stdin.read(1)
					if ch == '\n' or ch == '\r':
						break
				st = read_state(ser, addr, timeout=0.06)
				if st:
					_, x, y, btn, trig = st
					print(f"\r当前: X={x:4d} Y={y:4d} TRIG={trig:4d}      ", end="", flush=True)
			print()
			# 采样
			t0 = time.time()
			samples: List[Tuple[int,int,int,int,int]] = []
			while time.time() - t0 < duration_s:
				st = read_state(ser, addr, timeout=0.06)
				if st:
					samples.append(st)
				time.sleep(0.02)
			if not samples:
				return None
			# 均值（按字段求均值）
			ax = int(sum(s[1] for s in samples) / len(samples))
			ay = int(sum(s[2] for s in samples) / len(samples))
			ab = int(sum(s[3] for s in samples) / len(samples))
			at = int(sum(s[4] for s in samples) / len(samples))
			return (samples[-1][0], ax, ay, ab, at)
		finally:
			if old_term is not None:
				try:
					termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
				except Exception:
					pass

	# 1) 轻微左滑 -> 回弹静止 -> 采样1秒（均值）
	print("[Smart] 步骤1：轻微左滑摇杆后松开，确认回弹静止。")
	res_left = sample_mean(1.0)
	if not res_left:
		print("[ERR] 左滑静止采样失败")
		return 2
	_, x_left, y_left_echo, _, trig_l = res_left
	beep()

	# 2) 轻微右滑 -> 回弹静止 -> 采样1秒（均值）
	print("[Smart] 步骤2：轻微右滑摇杆后松开，确认回弹静止。")
	res_right = sample_mean(1.0)
	if not res_right:
		print("[ERR] 右滑静止采样失败")
		return 2
	_, x_right, y_right_echo, _, trig_r = res_right
	beep()

	# 根据左右静止均值，确定 X 中心；记录 TriggerStart 参考（左右均值平均）
	est_cx = int(round((x_left + x_right) / 2.0))
	ref_trig_start = int(round((trig_l + trig_r) / 2.0))
	print(f"[Smart] X_center={est_cx} (由左右两次静止均值决定)，TriggerStart≈{ref_trig_start}")
	beep()

	# 3) 轻微上滑 -> 回弹静止；轻微下滑 -> 回弹静止；各采样1秒，确定 Y 中心
	print("[Smart] 步骤3：轻微上滑后松开，确认回弹静止。")
	res_up = sample_mean(1.0)
	if not res_up:
		print("[ERR] 上滑静止采样失败")
		return 3
	_, x_up_echo, y_up, _, trig_u = res_up
	print("[Smart] 步骤4：轻微下滑后松开，确认回弹静止。")
	res_down = sample_mean(1.0)
	if not res_down:
		print("[ERR] 下滑静止采样失败")
		return 3
	_, x_down_echo, y_down, _, trig_d = res_down
	est_cy = int(round((y_up + y_down) / 2.0))
	print(f"[Smart] Y_center={est_cy} (由上下两次静止均值决定)")
	beep()

	# 5) 定圈覆盖采样，用于确定 X/Y 的 min/max
	print("[Smart] 步骤5：请绕圈滑动摇杆，尽可能覆盖所有方向，6秒后自动结束...")
	min_x, max_x = 0xFFFF, 0
	min_y, max_y = 0xFFFF, 0
	t0 = time.time()
	while time.time() - t0 < 6.0:
		st = read_state(ser, addr, timeout=0.06)
		if st:
			_, x, y, _, _ = st
			if x < min_x: min_x = x
			if x > max_x: max_x = x
			if y < min_y: min_y = y
			if y > max_y: max_y = y
		time.sleep(0.01)
	if min_x > max_x or min_y > max_y:
		print("[ERR] 覆盖采样失败，请重试")
		return 3
	print(f"[Smart] 极值范围：X=({min_x},{max_x})；Y=({min_y},{max_y})")
	beep()

	# 6) 板机最大
	print("[Smart] 步骤6：请将板机捏到底（最大），5秒后采样完成...")
	trig_max_samples = []
	t0 = time.time()
	while time.time() - t0 < 5.0:
		st = read_state(ser, addr, timeout=0.06)
		if st:
			_, _, _, _, trig = st
			trig_max_samples.append(trig)
		time.sleep(0.02)
	if not trig_max_samples:
		print("[ERR] 未采到板机样本")
		return 4
	est_trig_max = int(max(trig_max_samples))
	print(f"[Smart] TriggerMax≈{est_trig_max}，TriggerStart≈{ref_trig_start}")
	beep()

	# 7) 写入寄存器（直接写：0x30..0x37，2B BE），加小延迟
	def w16(reg, val):
		val = int(max(0, min(0xFFFF, val)))
		acks = write_register(ser, addr, reg, bytes([(val >> 8) & 0xFF, val & 0xFF]), timeout=0.25)
		ok = (addr == ADDR_BROADCAST and len(acks) > 0) or (addr in acks)
		time.sleep(0.05)
		return ok
	ok_all = True
	ok_all &= w16(0x30, est_cx)  # REG_ADDR_SET_JX_CENTER
	ok_all &= w16(0x31, max_x)   # REG_ADDR_SET_JX_MAX
	ok_all &= w16(0x32, min_x)   # REG_ADDR_SET_JX_MIN
	ok_all &= w16(0x33, est_cy)  # REG_ADDR_SET_JY_CENTER
	ok_all &= w16(0x34, max_y)   # REG_ADDR_SET_JY_MAX
	ok_all &= w16(0x35, min_y)   # REG_ADDR_SET_JY_MIN
	ok_all &= w16(0x36, ref_trig_start)  # REG_ADDR_SET_TRIG_START
	ok_all &= w16(0x37, est_trig_max)    # REG_ADDR_SET_TRIG_MAX
	print("[Smart] 写入标定 " + ("成功" if ok_all else "存在失败项"))
	beep()

	# 8) 质检：打开映射，沿用工厂质检逻辑
	print("[Smart] 步骤8：进入质检，打开映射输出")
	acks = write_register(ser, addr, REG_ADDR_CALIB_OUTPUT_ENABLE, bytes([1]), timeout=0.3)
	ok = (addr == ADDR_BROADCAST and len(acks) > 0) or (addr in acks)
	if not ok:
		print("[ERR] 打开映射输出失败，终止质检")
		return 5
	beep()

	print("请按以下姿态依次进行，并在每个动作稳定后按回车采样：")
	def wait_enter(label: str) -> Optional[Tuple[int,int,int,int,int]]:
		print(f" - {label}：稳定后按回车采样")
		old_term = None
		try:
			old_term = termios.tcgetattr(sys.stdin.fileno())
			tty.setcbreak(sys.stdin.fileno())
		except Exception:
			old_term = None
		sample = None
		try:
			while True:
				st = read_state(ser, addr, timeout=0.08)
				if st is not None:
					_, x, y, btn, trig = st
					print(f"\rX={x:4d}  Y={y:4d}  TRIG={trig:4d}  BTN=0x{btn:02X}      ", end="", flush=True)
					sample = st
				else:
					print("\r(无数据)                                           ", end="", flush=True)
				r, _, _ = select.select([sys.stdin], [], [], 0.02)
				if r:
					ch = sys.stdin.read(1)
					if ch == '\n' or ch == '\r':
						break
				time.sleep(0.02)
		finally:
			print()
			if old_term is not None:
				try:
					termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
				except Exception:
					pass
		return sample

	TOL = 120
	targets = [
		("静止(中心/起始)", 2048, 2048, 512),
		("板机最大", None, None, 3584),
	]
	results = []
	all_ok = True
	for label, tx, ty, tt in targets:
		s = wait_enter(label)
		if s is None:
			print("[ERR] 未采集到数据，终止")
			return 6
		_, x, y, _, trig = s
		if tx is not None:
			ok_x, msg_x = qc_check_value(f"{label}/X", x, tx, TOL); results.append(msg_x); all_ok &= ok_x
		if ty is not None:
			ok_y, msg_y = qc_check_value(f"{label}/Y", y, ty, TOL); results.append(msg_y); all_ok &= ok_y
		if tt is not None:
			ok_t, msg_t = qc_check_value(f"{label}/TRIG", trig, tt, TOL); results.append(msg_t); all_ok &= ok_t

	# 追加：旋转6秒极值范围质检（不替代姿态质检）
	print("旋转质检：请绕圈持续旋转摇杆约6秒，检测输出极值范围。")
	t0 = time.time()
	qc_min_x, qc_max_x = 0xFFFF, 0
	qc_min_y, qc_max_y = 0xFFFF, 0
	while time.time() - t0 < 6.0:
		st = read_state(ser, addr, timeout=0.08)
		if st is not None:
			_, x, y, _, _ = st
			if x < qc_min_x: qc_min_x = x
			if x > qc_max_x: qc_max_x = x
			if y < qc_min_y: qc_min_y = y
			if y > qc_max_y: qc_max_y = y
		time.sleep(0.01)
	# 追加极值质检结果
	ok1, msg1 = qc_check_value("X 左最大/X", qc_max_x, 3848, TOL); results.append(msg1); all_ok &= ok1
	ok2, msg2 = qc_check_value("X 右最小/X", qc_min_x, 248, TOL);  results.append(msg2); all_ok &= ok2
	ok3, msg3 = qc_check_value("Y 上最大/Y", qc_max_y, 3848, TOL); results.append(msg3); all_ok &= ok3
	ok4, msg4 = qc_check_value("Y 下最小/Y", qc_min_y, 248, TOL);  results.append(msg4); all_ok &= ok4

	print("\n[Smart] 质检结果")
	for line in results:
		print("  - " + line)
	print("\n最终结果: " + ("PASS" if all_ok else "FAIL"))
	beep(2)
	return 0 if all_ok else 7


def run_smart_calib_guarded(ser: serial.Serial, addr: int):
	try:
		return run_smart_calib(ser, addr)
	except KeyboardInterrupt:
		print("\n[Smart] 检测到中断(Ctrl+C)，尝试开启校准输出以避免长期关闭...")
		ok = False
		try:
			acks = write_register(ser, addr, REG_ADDR_CALIB_OUTPUT_ENABLE, bytes([1]), timeout=0.3)
			ok = (addr == ADDR_BROADCAST and len(acks) > 0) or (addr in acks)
		except Exception:
			ok = False
		if ok:
			print("[Smart] 已成功重新打开校准输出")
		else:
			print("[ERR] 未能重新打开校准输出，请手动检查并打开")
		return 1
def cmd_poll(args):
	port = args.port or detect_default_port()
	if not port:
		print("未找到串口，请使用 --port 指定，如 /dev/tty.usbserial-xxxx")
		return 1
	dev_id = args.id & 0x0F
	baud = args.baud
	interval = max(0.02, args.interval)
	print(f"[INFO] 打开串口 {port} @ {baud}，轮询设备ID={dev_id}")
	with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
		try:
			while True:
				result = read_state(ser, dev_id, timeout=0.08)
				if result is not None:
					_, x, y, btn, trig = result
					print(f"X={x:4d}  Y={y:4d}  TRIG={trig:4d}  BTN=0x{btn:02X}", end="\r", flush=True)
				else:
					print("(无数据)                                     ", end="\r", flush=True)
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
		"jx_center": REG_ADDR_READ_JX_CENTER,
		"jx_max": REG_ADDR_READ_JX_MAX,
		"jx_min": REG_ADDR_READ_JX_MIN,
		"jy_center": REG_ADDR_READ_JY_CENTER,
		"jy_max": REG_ADDR_READ_JY_MAX,
		"jy_min": REG_ADDR_READ_JY_MIN,
		"trig_start": REG_ADDR_READ_TRIG_START,
		"trig_max": REG_ADDR_READ_TRIG_MAX,
		"calib_on": REG_ADDR_CALIB_OUTPUT_ENABLE,
		"state": None,
	}
	if args.reg not in reg_map:
		print("不支持的寄存器")
		return 2
	reg = reg_map[args.reg]
	baud = args.baud
	with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
		if reg is None:
			res = read_state(ser, addr, timeout=0.2)
			if res is None:
				print("[ERR] 无响应")
				return 2
			raddr, x, y, btn, trig = res
			print(f"[RESP] from id={raddr}: X={x} Y={y} BTN=0x{btn:02X} TRIG={trig}")
			return 0
		else:
			res = read_register(ser, addr, reg, timeout=0.25)
			if not res:
				print("[ERR] 无响应")
				return 2
			for (raddr, data) in res:
				if reg == REG_ADDR_VERSION and len(data) >= 4:
					ver = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
					print(f"[RESP] from id={raddr}: version=0x{ver:08X}")
				elif reg == REG_ADDR_DEVICE_ID and len(data) >= 1:
					print(f"[RESP] from id={raddr}: device_id={data[0] & 0x0F}")
				elif reg == REG_ADDR_CALIB_OUTPUT_ENABLE and len(data) >= 1:
					print(f"[RESP] from id={raddr}: calib_output={'ON' if (data[0]&1) else 'OFF'}")
				elif len(data) >= 2:
					val = (data[0] << 8) | data[1]
					print(f"[RESP] from id={raddr}: {args.reg}={val}")
				else:
					print(f"[RESP] from id={raddr}: raw={data.hex()}")
	return 0


def cmd_write(args):
	port = args.port or detect_default_port()
	if not port:
		print("未找到串口，请使用 --port 指定")
		return 1
	addr = ADDR_BROADCAST if args.broadcast else (args.id & 0x0F)
	baud = args.baud
	reg_map = {
		"calib_on": REG_ADDR_CALIB_OUTPUT_ENABLE,
		"x_center_here": REG_ADDR_SET_JX_CENTER_CURRENT,
		"x_max_here": REG_ADDR_SET_JX_MAX_CURRENT,
		"x_min_here": REG_ADDR_SET_JX_MIN_CURRENT,
		"y_center_here": REG_ADDR_SET_JY_CENTER_CURRENT,
		"y_max_here": REG_ADDR_SET_JY_MAX_CURRENT,
		"y_min_here": REG_ADDR_SET_JY_MIN_CURRENT,
		"trig_start_here": REG_ADDR_SET_TRIG_START_CURRENT,
		"trig_max_here": REG_ADDR_SET_TRIG_MAX_CURRENT,
	}
	if args.reg not in reg_map:
		print("不支持写寄存器")
		return 2
	reg = reg_map[args.reg]
	with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
		if reg == REG_ADDR_CALIB_OUTPUT_ENABLE:
			if args.value is None:
				print("需要 --value 0/1")
				return 2
			v = 1 if str(args.value).strip() not in ("0", "off", "OFF") else 0
			acks = write_register(ser, addr, reg, bytes([v]), timeout=0.3)
		else:
			acks = write_register(ser, addr, reg, b"", timeout=0.3)
		if not acks:
			print("[ERR] 未收到写入响应")
			return 3
		if addr == ADDR_BROADCAST:
			ids = ", ".join(str(a) for a in sorted(set(acks)))
			print(f"[OK] 广播写入已响应的设备: {ids}")
		else:
			print(f"[OK] 设备 {addr} 已确认写入")
	return 0


def cmd_factory(args):
	port = args.port or detect_default_port()
	if not port:
		print("未找到串口，请使用 --port 指定，如 /dev/tty.usbserial-xxxx")
		return 1
	addr = args.id & 0x0F
	baud = args.baud
	with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
		return run_factory(ser, addr)


def cmd_calib_joystick(args):
	port = args.port or detect_default_port()
	if not port:
		print("未找到串口，请使用 --port 指定，如 /dev/tty.usbserial-xxxx")
		return 1
	addr = args.id & 0x0F
	baud = args.baud
	with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
		return run_calib_joystick(ser, addr)


def cmd_calib_trigger(args):
	port = args.port or detect_default_port()
	if not port:
		print("未找到串口，请使用 --port 指定，如 /dev/tty.usbserial-xxxx")
		return 1
	addr = args.id & 0x0F
	baud = args.baud
	with serial.Serial(port=port, baudrate=baud, timeout=0) as ser:
		return run_calib_trigger(ser, addr)


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

	current_id = 9  # 默认与固件一致
	polling = False
	poll_interval = 0.05
	print("\n进入交互模式。快捷键：")
	print("  0) 智能校准（默认/推荐，直接回车进入）")
	print("  1) 切换轮询状态 开/关")
	print("  2) 设置当前设备ID")
	print("  3) 读取标定寄存器与开关")
	print("  4) 设置校准输出开关(0/1)")
	print("  5) 工厂模式（完整校准并质检）")
	print("  6) 仅校准摇杆")
	print("  7) 仅校准板机")
	print("  8) 读取软件版本号")
	print("  a) 将当前位置设为X/Y中心、TRIGGER起始（逐项）")
	print("  m) 将当前位置设为X/Y/TRIGGER极值（逐项提示）")
	print("  p) 选择串口/波特率")
	print("  q) 退出")

	old_term = None
	quit_all = False
	try:
		while True:
			if polling:
				# 进入轮询时设置终端为cbreak，支持按键即时停止
				if old_term is None:
					try:
						old_term = termios.tcgetattr(sys.stdin.fileno())
						tty.setcbreak(sys.stdin.fileno())
						print("\n[轮询中] 按 1 停止轮询，按 q 退出工具。", flush=True)
					except Exception:
						old_term = None

				result = read_state(ser, current_id, timeout=0.08)
				if result is not None:
					rid, x, y, btn, trig = result
					label_id = current_id
					print(f"[ID={label_id}] X={x:4d}  Y={y:4d}  TRIG={trig:4d}  BTN=0x{btn:02X}", end="\r", flush=True)
				else:
					print(f"[ID={current_id}] (无数据)                                     ", end="\r", flush=True)

				# 检查键盘
				try:
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
					if old_term is not None:
						try:
							termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
						except Exception:
							pass
						old_term = None
					print()  # 换行
					if quit_all:
						break
			else:
				cmd = input("\n选择操作(0/1/2/3/4/5/6/7/8/a/m/p/q)：").strip().lower()
				if cmd == "":
					cmd = "0"
				if cmd == "q":
					break
				elif cmd == "0":
					run_smart_calib_guarded(ser, current_id)
				elif cmd == "1":
					polling = not polling
					print(f"轮询={'开启' if polling else '关闭'} (ID={current_id})")
				elif cmd == "2":
					try:
						nid = int(input("输入新设备ID(0~15)：").strip(), 0) & 0x0F
						current_id = nid
						print(f"[OK] 切换脚本目标ID={current_id}")
					except Exception as e:
						print(f"[ERR] {e}")
				elif cmd == "3":
					# 读取部分关键寄存器
					def read_u16(reg, name):
						res = read_register(ser, current_id, reg, timeout=0.2)
						if not res: print(f"  {name}: (无响应)"); return
						_, data = res[0]
						if len(data) >= 2:
							val = (data[0] << 8) | data[1]
							print(f"  {name}: {val}")
						elif len(data) == 1:
							print(f"  {name}: {data[0]}")
						else:
							print(f"  {name}: raw={data.hex()}")
					print("[READ] 标定寄存器：")
					read_u16(REG_ADDR_READ_JX_CENTER, "jx_center")
					read_u16(REG_ADDR_READ_JX_MAX, "jx_max")
					read_u16(REG_ADDR_READ_JX_MIN, "jx_min")
					read_u16(REG_ADDR_READ_JY_CENTER, "jy_center")
					read_u16(REG_ADDR_READ_JY_MAX, "jy_max")
					read_u16(REG_ADDR_READ_JY_MIN, "jy_min")
					read_u16(REG_ADDR_READ_TRIG_START, "trig_start")
					read_u16(REG_ADDR_READ_TRIG_MAX, "trig_max")
					res = read_register(ser, current_id, REG_ADDR_CALIB_OUTPUT_ENABLE, timeout=0.2)
					if res and len(res[0][1]) >= 1:
						print(f"  calib_on: {res[0][1][0] & 1}")
				elif cmd == "4":
					try:
						v = input("设置校准输出(0/1)：").strip()
						vb = 1 if v not in ("0", "off", "OFF") else 0
						acks = write_register(ser, current_id, REG_ADDR_CALIB_OUTPUT_ENABLE, bytes([vb]), timeout=0.3)
						print("[OK]" if ((current_id in acks) or (len(acks) > 0)) else "[ERR] 无响应")
					except Exception as e:
						print(f"[ERR] {e}")
				elif cmd == "5":
					run_factory(ser, current_id)
				elif cmd == "6":
					run_calib_joystick(ser, current_id)
				elif cmd == "7":
					run_calib_trigger(ser, current_id)
				elif cmd == "8":
					# 读取软件版本号
					try:
						res = read_register(ser, current_id, REG_ADDR_VERSION, timeout=0.25)
						if not res:
							print("[ERR] 读取版本失败：无响应")
						else:
							raddr, data = res[0]
							if len(data) >= 4:
								ver = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
								print(f"[RESP] from id={raddr}: version=0x{ver:08X}")
							else:
								print(f"[RESP] from id={raddr}: raw={data.hex()}")
					except Exception as e:
						print(f"[ERR] 读取版本异常: {e}")
				elif cmd == "a":
					prompt_live_and_commit(ser, current_id, "静止 -> 设 X 中心", REG_ADDR_SET_JX_CENTER_CURRENT)
					prompt_live_and_commit(ser, current_id, "静止 -> 设 Y 中心", REG_ADDR_SET_JY_CENTER_CURRENT)
					prompt_live_and_commit(ser, current_id, "静止 -> 设 TRIGGER 起始", REG_ADDR_SET_TRIG_START_CURRENT)
				elif cmd == "m":
					prompt_live_and_commit(ser, current_id, "推到 X 最大（左）", REG_ADDR_SET_JX_MAX_CURRENT)
					prompt_live_and_commit(ser, current_id, "推到 X 最小（右）", REG_ADDR_SET_JX_MIN_CURRENT)
					prompt_live_and_commit(ser, current_id, "推到 Y 最大（上）", REG_ADDR_SET_JY_MAX_CURRENT)
					prompt_live_and_commit(ser, current_id, "推到 Y 最小（下）", REG_ADDR_SET_JY_MIN_CURRENT)
					prompt_live_and_commit(ser, current_id, "捏住板机至最大", REG_ADDR_SET_TRIG_MAX_CURRENT)
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
				else:
					print("未知操作")
	finally:
		try:
			if old_term is not None:
				try:
					termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
				except Exception:
					pass
			ser.close()
		except Exception:
			pass
	return 0


def build_parser() -> argparse.ArgumentParser:
	p = argparse.ArgumentParser(description="Handler RS485 通信与校准工具")
	p.add_argument("--port", help="串口设备路径，如 /dev/tty.usbserial-xxxx")
	p.add_argument("--baud", type=int, default=2000000, help="波特率，默认 2000000")

	sub = p.add_subparsers(dest="cmd")
	common = argparse.ArgumentParser(add_help=False)
	common.add_argument("--port", help="串口设备路径，如 /dev/tty.usbserial-xxxx")
	common.add_argument("--baud", type=int, help="波特率(覆盖全局)")

	sp_smart = sub.add_parser("smart", parents=[common], help="智能校准：自动采样极值与中心并写入，再质检")
	sp_smart.add_argument("--id", type=int, required=True, help="设备ID(0~15)")
	sp_smart.set_defaults(func=cmd_factory if False else (lambda args: (lambda: (  # hack to avoid reordering
		(lambda port_baud: (lambda port, baud: (  # inline open and call
			(lambda ser: (run_smart_calib_guarded(ser, args.id & 0x0F) if ser else 1))(serial.Serial(port=port, baudrate=baud, timeout=0) if port else None)
		))(*port_baud)
	))( (args.port or detect_default_port(), args.baud or 2000000) )
	)() ) )  # keeps simple without duplicating too much code

	sp_poll = sub.add_parser("poll", parents=[common], help="根据设备ID持续读取状态(X/Y/Trigger/Buttons)")
	sp_poll.add_argument("--id", type=int, required=True, help="设备ID(0~15)")
	sp_poll.add_argument("--interval", type=float, default=0.05, help="读取间隔秒，默认0.05s")
	sp_poll.set_defaults(func=cmd_poll)

	sp_read = sub.add_parser("read", parents=[common], help="读取寄存器或状态")
	sp_read.add_argument("--id", type=int, help="设备ID(0~15)，若不指定且 --broadcast 则为广播")
	sp_read.add_argument("--broadcast", action="store_true", help="使用广播地址0读取")
	sp_read.add_argument("--reg", choices=[
		"version","id",
		"jx_center","jx_max","jx_min",
		"jy_center","jy_max","jy_min",
		"trig_start","trig_max",
		"calib_on","state"
	], required=True, help="寄存器或状态")
	sp_read.set_defaults(func=cmd_read)

	sp_write = sub.add_parser("write", parents=[common], help="写寄存器（支持动作型与校准开关）")
	sp_write.add_argument("--id", type=int, required=True, help="设备ID(0~15)")
	sp_write.add_argument("--reg", choices=[
		"calib_on",
		"x_center_here","x_max_here","x_min_here",
		"y_center_here","y_max_here","y_min_here",
		"trig_start_here","trig_max_here"
	], required=True, help="寄存器")
	sp_write.add_argument("--value", help="仅 calib_on 需要：0/1")
	sp_write.set_defaults(func=cmd_write)

	sp_factory = sub.add_parser("factory", parents=[common], help="工厂模式：完整校准并质检")
	sp_factory.add_argument("--id", type=int, required=True, help="设备ID(0~15)")
	sp_factory.set_defaults(func=cmd_factory)

	sp_cal_j = sub.add_parser("calib_joystick", parents=[common], help="单独校准摇杆")
	sp_cal_j.add_argument("--id", type=int, required=True, help="设备ID(0~15)")
	sp_cal_j.set_defaults(func=cmd_calib_joystick)

	sp_cal_t = sub.add_parser("calib_trigger", parents=[common], help="单独校准板机")
	sp_cal_t.add_argument("--id", type=int, required=True, help="设备ID(0~15)")
	sp_cal_t.set_defaults(func=cmd_calib_trigger)

	sp_ui = sub.add_parser("ui", parents=[common], help="交互式面板：轮询、校准、寄存器读写")
	sp_ui.set_defaults(func=cmd_ui)

	return p


def main(argv: List[str]) -> int:
	parser = build_parser()
	if len(argv) == 0:
		parser.print_help()
		return 0
	args = parser.parse_args(argv)
	if not hasattr(args, "func"):
		parser.print_help()
		return 0
	try:
		return args.func(args)
	except KeyboardInterrupt:
		print("\n[INFO] 退出")
		return 0


if __name__ == "__main__":
	sys.exit(main(sys.argv[1:]))


