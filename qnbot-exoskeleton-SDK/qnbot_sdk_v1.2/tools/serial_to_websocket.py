#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import asyncio
import json
import logging
import math
import queue
import signal
import struct
import sys
import threading
import time
from dataclasses import asdict
from dataclasses import dataclass
from dataclasses import field

import serial

try:
    import websockets
except ImportError as exc:
    raise SystemExit(
        "缺少依赖 websockets，请先安装：pip install websockets"
    ) from exc


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("SerialWebSocketServer")


REPORT_FRAME_HEADER = 0xAA
REPORT_FRAME_TAIL = 0x55
DATA_LEN_BASE = 48
DATA_LEN_TORSO_IMU_QUAT = 88
DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT = 128
SUPPORTED_FRAME_LENGTHS = [
    1 + DATA_LEN_BASE + 2,
    1 + DATA_LEN_TORSO_IMU_QUAT + 2,
    1 + DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT + 2,
]
ENCODER_TO_RADIAN_RATIO = 2 * math.pi / 16384


@dataclass
class RemoteManipulatorData:
    timestamp: float = 0.0
    joystick_left: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    joystick_right: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    arm_joint_left: list[int] = field(default_factory=lambda: [0] * 8)
    arm_joint_right: list[int] = field(default_factory=lambda: [0] * 8)
    arm_joint_left_rad: list[float] = field(default_factory=lambda: [0.0] * 8)
    arm_joint_right_rad: list[float] = field(default_factory=lambda: [0.0] * 8)
    torso_acc: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    torso_gyro: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    torso_quat: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    extra_acc: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    extra_gyro: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    extra_quat: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    format_version: int = 1


class SerialWebSocketServer:
    def __init__(
        self,
        serial_device: str,
        baudrate: int,
        ws_host: str,
        ws_port: int,
        device_id: str,
    ):
        self.serial_device = serial_device
        self.baudrate = baudrate
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.device_id = device_id

        self.running = False
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.frame_buffer = bytearray()

        self.read_thread = None
        self.broadcast_queue = queue.Queue(maxsize=1)

        self.clients = set()
        self.clients_lock = threading.Lock()
        self.loop = None
        self.server = None

        self.frames_parsed = 0
        self.frames_broadcast = 0
        self.frames_dropped = 0
        self.bad_sync = 0

    def start(self) -> None:
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        asyncio.run(self._run_websocket_server())

    def stop(self) -> None:
        self.running = False

        with self.serial_lock:
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None

        if self.read_thread:
            self.read_thread.join(timeout=2.0)

        logger.info(
            "服务已停止，解析 %d 帧，广播 %d 帧，丢弃 %d 帧，失步 %d 次",
            self.frames_parsed,
            self.frames_broadcast,
            self.frames_dropped,
            self.bad_sync,
        )

    def _open_serial(self):
        while self.running:
            try:
                logger.info("正在打开串口: %s", self.serial_device)
                ser = serial.Serial(
                    port=self.serial_device,
                    baudrate=self.baudrate,
                    timeout=0.02,
                    write_timeout=0,
                )
                logger.info("串口已连接: %s", self.serial_device)
                return ser
            except Exception as exc:
                logger.warning("打开串口失败: %s", exc)
                time.sleep(1.0)
        return None

    def _read_loop(self) -> None:
        while self.running:
            with self.serial_lock:
                ser = self.serial_port

            if ser is None or not getattr(ser, "is_open", False):
                ser = self._open_serial()
                if ser is None:
                    break
                with self.serial_lock:
                    self.serial_port = ser

            try:
                in_waiting = ser.in_waiting
                if in_waiting > 0:
                    chunk = ser.read(min(in_waiting, 4096))
                else:
                    chunk = ser.read(1)
                    if chunk:
                        more = ser.read(ser.in_waiting)
                        if more:
                            chunk += more

                if chunk:
                    self._accumulate_and_process_frames(chunk)
                else:
                    time.sleep(0.001)
            except Exception as exc:
                logger.warning("串口读取异常，准备重连: %s", exc)
                with self.serial_lock:
                    try:
                        if self.serial_port is not None:
                            self.serial_port.close()
                    except Exception:
                        pass
                    self.serial_port = None
                time.sleep(0.5)

    def _accumulate_and_process_frames(self, chunk: bytes) -> None:
        self.frame_buffer.extend(chunk)
        min_frame_len = min(SUPPORTED_FRAME_LENGTHS)

        while len(self.frame_buffer) >= min_frame_len:
            header_index = self.frame_buffer.find(bytes([REPORT_FRAME_HEADER]))
            if header_index == -1:
                self.frame_buffer.clear()
                return

            if header_index > 0:
                self.frame_buffer = self.frame_buffer[header_index:]

            if len(self.frame_buffer) < min_frame_len:
                return

            parsed = False
            for frame_len in SUPPORTED_FRAME_LENGTHS:
                if len(self.frame_buffer) < frame_len:
                    continue

                frame = self.frame_buffer[:frame_len]
                if frame[-1] != REPORT_FRAME_TAIL:
                    continue

                checksum = 0
                for i in range(1, frame_len - 2):
                    checksum ^= frame[i]
                if checksum != frame[frame_len - 2]:
                    continue

                payload_len = frame_len - 3
                data = self._parse_frame(frame, payload_len)
                if data is None:
                    continue

                self.frames_parsed += 1
                self._enqueue_broadcast(self._build_message(data))
                self.frame_buffer = self.frame_buffer[frame_len:]
                parsed = True
                break

            if not parsed:
                self.frame_buffer = self.frame_buffer[1:]
                self.bad_sync += 1

    def _parse_frame(self, frame: bytes, payload_len: int) -> RemoteManipulatorData | None:
        try:
            payload = frame[1:-2]
            data = RemoteManipulatorData(timestamp=time.time())

            offset = 0
            for i in range(4):
                data.joystick_left[i] = struct.unpack("<h", payload[offset:offset + 2])[0]
                offset += 2

            for i in range(4):
                data.joystick_right[i] = struct.unpack("<h", payload[offset:offset + 2])[0]
                offset += 2

            for i in range(8):
                value = struct.unpack("<h", payload[offset:offset + 2])[0]
                data.arm_joint_left[i] = value
                data.arm_joint_left_rad[i] = value * ENCODER_TO_RADIAN_RATIO
                offset += 2

            for i in range(8):
                value = struct.unpack("<h", payload[offset:offset + 2])[0]
                data.arm_joint_right[i] = value
                data.arm_joint_right_rad[i] = value * ENCODER_TO_RADIAN_RATIO
                offset += 2

            if payload_len == DATA_LEN_BASE:
                data.format_version = 1
            elif payload_len == DATA_LEN_TORSO_IMU_QUAT:
                data.format_version = 2
                for i in range(3):
                    data.torso_acc[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
                for i in range(3):
                    data.torso_gyro[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
                for i in range(4):
                    data.torso_quat[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
            elif payload_len == DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT:
                data.format_version = 3
                for i in range(3):
                    data.torso_acc[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
                for i in range(3):
                    data.torso_gyro[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
                for i in range(4):
                    data.torso_quat[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
                for i in range(3):
                    data.extra_acc[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
                for i in range(3):
                    data.extra_gyro[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
                for i in range(4):
                    data.extra_quat[i] = struct.unpack("<f", payload[offset:offset + 4])[0]
                    offset += 4
            else:
                return None

            return data
        except Exception as exc:
            logger.warning("解析串口帧失败: %s", exc)
            return None

    def _build_message(self, data: RemoteManipulatorData) -> str:
        message = {
            "deviceId": self.device_id,
            "timestamp": int(data.timestamp * 1000),
            "type": "exoskeleton-serial",
            "data": asdict(data),
        }
        return json.dumps(message, ensure_ascii=False)

    def _enqueue_broadcast(self, payload: str) -> None:
        try:
            if self.broadcast_queue.full():
                self.broadcast_queue.get_nowait()
                self.frames_dropped += 1
            self.broadcast_queue.put_nowait(payload)
        except queue.Full:
            self.frames_dropped += 1

    async def _run_websocket_server(self) -> None:
        self.loop = asyncio.get_running_loop()
        self.server = await websockets.serve(
            self._handle_client,
            self.ws_host,
            self.ws_port,
        )
        logger.info(
            "WebSocket 服务已启动: ws://%s:%d",
            self.ws_host,
            self.ws_port,
        )

        broadcaster_task = asyncio.create_task(self._broadcast_loop())
        try:
            await self.server.wait_closed()
        finally:
            broadcaster_task.cancel()
            try:
                await broadcaster_task
            except asyncio.CancelledError:
                pass

    async def _handle_client(self, websocket):
        client = getattr(websocket, "remote_address", None)
        with self.clients_lock:
            self.clients.add(websocket)
            client_count = len(self.clients)
        logger.info("客户端已连接: %s, 当前连接数: %d", client, client_count)

        try:
            async for _ in websocket:
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            with self.clients_lock:
                self.clients.discard(websocket)
                client_count = len(self.clients)
            logger.info("客户端已断开: %s, 当前连接数: %d", client, client_count)

    async def _broadcast_loop(self) -> None:
        while self.running:
            try:
                payload = await asyncio.to_thread(self.broadcast_queue.get, True, 0.2)
            except queue.Empty:
                continue

            with self.clients_lock:
                clients = list(self.clients)

            if not clients:
                continue

            stale_clients = []
            for client in clients:
                try:
                    await client.send(payload)
                    self.frames_broadcast += 1
                except Exception:
                    stale_clients.append(client)

            if stale_clients:
                with self.clients_lock:
                    for client in stale_clients:
                        self.clients.discard(client)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取串口外骨骼数据并作为 WebSocket 服务端广播"
    )
    parser.add_argument("--serial-device", default="/dev/tty.usbmodemCMSIS_DAP2", help="串口设备路径")
    parser.add_argument("--baudrate", type=int, default=2000000, help="串口波特率")
    parser.add_argument("--host", default="0.0.0.0", help="WebSocket 服务监听地址")
    parser.add_argument("--port", type=int, default=19091, help="WebSocket 服务监听端口")
    parser.add_argument("--device-id", default="qnbot-exoskeleton", help="发送消息里的 deviceId")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    server = SerialWebSocketServer(
        serial_device=args.serial_device,
        baudrate=args.baudrate,
        ws_host=args.host,
        ws_port=args.port,
        device_id=args.device_id,
    )

    def handle_stop(_signum=None, _frame=None):
        logger.info("收到退出信号，正在停止...")
        if server.server is not None:
            server.server.close()
        server.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    try:
        server.start()
    except KeyboardInterrupt:
        handle_stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
