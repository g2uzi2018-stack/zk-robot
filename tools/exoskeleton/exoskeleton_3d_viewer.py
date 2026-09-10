#!/usr/bin/env python3
"""Serve a low-poly 3D mechanical view of the exoskeleton telemetry.

The live reader uses the same legacy frame layout as the project's other
read-only monitor.  The browser view is deliberately an approximate diagnostic
model of a seven-actuator robot arm: slots 0/1 are shoulder axes, slot 2 is an
upper-arm roll, slot 3 is elbow flexion, slot 4 is forearm roll, and slots 5/6
are two wrist axes.  Slot 7 is shown as an unmapped diagnostic value and does
not affect the pose.  The pose renderer uses a wearer frame (+X right, +Y
front, +Z up) and applies the recorded left/right direction signs as a
diagnostic visualization mapping. The
wearer-view record is treated as authoritative, including the confirmed forward
direction of right slot 0.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import threading
import time
import webbrowser
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

from exoskeleton_serial import format_usb_id, parse_usb_id, resolve_serial_port


FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55
PAYLOAD_LENGTHS = (128, 88, 48)
FRAME_LENGTHS = tuple(length + 3 for length in PAYLOAD_LENGTHS)
JOINT_COUNT = 8
POSE_JOINT_COUNT = 7
ENCODER_TO_RADIAN_RATIO = 2.0 * math.pi / 16384.0


@dataclass(frozen=True)
class Snapshot:
    frame_size: int
    format_version: int
    left_raw: tuple[int, ...]
    right_raw: tuple[int, ...]

    @property
    def left_rad(self) -> tuple[float, ...]:
        return tuple(value * ENCODER_TO_RADIAN_RATIO for value in self.left_raw)

    @property
    def right_rad(self) -> tuple[float, ...]:
        return tuple(value * ENCODER_TO_RADIAN_RATIO for value in self.right_raw)


def parse_frame(frame: bytes) -> Snapshot:
    """Parse one complete, validated legacy frame."""
    if len(frame) not in FRAME_LENGTHS:
        raise ValueError(f"unsupported legacy frame length: {len(frame)}")
    if frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
        raise ValueError("legacy frame header or tail is invalid")

    checksum = 0
    for value in frame[1:-2]:
        checksum ^= value
    if checksum != frame[-2]:
        raise ValueError("legacy frame checksum is invalid")

    payload = frame[1:-2]
    payload_length = len(payload)
    left = struct.unpack_from("<8h", payload, 16)
    right = struct.unpack_from("<8h", payload, 32)
    return Snapshot(
        frame_size=len(frame),
        format_version={48: 1, 88: 2, 128: 3}[payload_length],
        left_raw=tuple(left),
        right_raw=tuple(right),
    )


class LegacySerialReader:
    """Read and reassemble the Qnbot legacy telemetry stream."""

    def __init__(self, port: str, baudrate: int) -> None:
        try:
            import serial
        except ImportError as error:
            raise RuntimeError(
                "live mode requires pyserial; install it with "
                "python3 -m pip install pyserial"
            ) from error

        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.02,
        )
        self._buffer = bytearray()

    def close(self) -> None:
        self._serial.close()

    def read_snapshot(self) -> Optional[Snapshot]:
        chunk = self._serial.read(self._serial.in_waiting or 1)
        if chunk:
            self._buffer.extend(chunk)
        return self._extract_snapshot()

    def _extract_snapshot(self) -> Optional[Snapshot]:
        while True:
            try:
                head = self._buffer.index(FRAME_HEADER)
            except ValueError:
                self._buffer.clear()
                return None

            if head:
                del self._buffer[:head]
            if len(self._buffer) < min(FRAME_LENGTHS):
                return None

            needs_more_data = False
            for frame_length in FRAME_LENGTHS:
                if len(self._buffer) < frame_length:
                    needs_more_data = True
                    continue

                candidate = bytes(self._buffer[:frame_length])
                if candidate[-1] != FRAME_TAIL:
                    continue
                checksum = 0
                for value in candidate[1:-2]:
                    checksum ^= value
                if checksum != candidate[-2]:
                    continue

                del self._buffer[:frame_length]
                return parse_frame(candidate)

            if needs_more_data:
                return None
            del self._buffer[0]


class TelemetrySource:
    """Own the serial/demo loop so the HTTP handlers stay non-blocking."""

    def __init__(
        self,
        port: Optional[str],
        vid: int,
        pid: int,
        baudrate: int,
        demo: bool,
        relative_pose: bool,
        stale_timeout: float,
    ) -> None:
        self._port = port
        self._vid = vid
        self._pid = pid
        self._baudrate = baudrate
        self._demo = demo
        self._relative_pose = relative_pose
        self._stale_timeout = stale_timeout
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._snapshot: Optional[Snapshot] = None
        self._pose_left: tuple[float, ...] = (0.0,) * JOINT_COUNT
        self._pose_right: tuple[float, ...] = (0.0,) * JOINT_COUNT
        self._baseline_left: Optional[tuple[float, ...]] = None
        self._baseline_right: Optional[tuple[float, ...]] = None
        self._last_update = 0.0
        self._connected = False
        self._active_port: Optional[str] = None
        self._status = "等待遥测数据"

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="exoskeleton-3d-source",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None

    def state(self) -> dict[str, object]:
        now = time.monotonic()
        with self._lock:
            snapshot = self._snapshot
            age = now - self._last_update if self._last_update else None
            fresh = (
                snapshot is not None
                and self._connected
                and age is not None
                and 0.0 <= age <= self._stale_timeout
            )
            return {
                "source": "demo" if self._demo else "serial",
                "status": self._status,
                "connected": self._connected,
                "usb_id": format_usb_id(self._vid, self._pid),
                "port": self._active_port or self._port,
                "fresh": fresh,
                "age_ms": None if age is None else max(0.0, age * 1000.0),
                "frame_size": 0 if snapshot is None else snapshot.frame_size,
                "format_version": 0
                if snapshot is None
                else snapshot.format_version,
                "relative_pose": self._relative_pose,
                "left_raw": list((0,) * JOINT_COUNT if snapshot is None else snapshot.left_raw),
                "right_raw": list((0,) * JOINT_COUNT if snapshot is None else snapshot.right_raw),
                "left_rad": list(
                    (0.0,) * JOINT_COUNT if snapshot is None else snapshot.left_rad
                ),
                "right_rad": list(
                    (0.0,) * JOINT_COUNT if snapshot is None else snapshot.right_rad
                ),
                "left_pose_rad": list(self._pose_left),
                "right_pose_rad": list(self._pose_right),
            }

    def _run(self) -> None:
        if self._demo:
            self._run_demo()
            return

        reader: Optional[LegacySerialReader] = None
        try:
            while not self._stop.is_set():
                if reader is None:
                    self._set_status("正在连接串口", connected=False, reset_baseline=True)
                    try:
                        active_port = resolve_serial_port(
                            self._port,
                            self._vid,
                            self._pid,
                        )
                        reader = LegacySerialReader(active_port, self._baudrate)
                    except Exception as error:  # serial errors are runtime state
                        self._set_status(
                            f"串口等待中：{error}",
                            connected=False,
                            reset_baseline=True,
                        )
                        self._stop.wait(1.0)
                        continue
                    self._set_status(
                        f"串口已打开（{active_port}），等待 legacy 遥测",
                        connected=True,
                        active_port=active_port,
                    )

                try:
                    snapshot = reader.read_snapshot()
                except Exception as error:  # hot-unplug/reconnect path
                    reader.close()
                    reader = None
                    self._set_status(
                        f"串口断开，准备重连：{error}",
                        connected=False,
                        reset_baseline=True,
                    )
                    self._stop.wait(0.2)
                    continue

                if snapshot is not None:
                    self._publish(snapshot)
                else:
                    self._stop.wait(0.01)
        finally:
            if reader is not None:
                reader.close()

    def _run_demo(self) -> None:
        started = time.monotonic()
        self._set_status("离线演示", connected=True, reset_baseline=True)
        while not self._stop.is_set():
            elapsed = time.monotonic() - started
            left = tuple(
                0.55 * math.sin(elapsed * 0.9 + index * 0.31)
                for index in range(POSE_JOINT_COUNT)
            ) + (0.0,)
            right = tuple(
                0.48 * math.sin(elapsed * 0.9 + index * 0.31 + 1.2)
                for index in range(POSE_JOINT_COUNT)
            ) + (0.0,)
            self._publish(
                Snapshot(
                    frame_size=131,
                    format_version=3,
                    left_raw=tuple(self._radian_to_raw(value) for value in left),
                    right_raw=tuple(self._radian_to_raw(value) for value in right),
                )
            )
            self._stop.wait(0.033)

    @staticmethod
    def _radian_to_raw(value: float) -> int:
        return max(-32768, min(32767, int(round(value / ENCODER_TO_RADIAN_RATIO))))

    def _publish(self, snapshot: Snapshot) -> None:
        left_rad = snapshot.left_rad
        right_rad = snapshot.right_rad
        with self._lock:
            if self._baseline_left is None or self._baseline_right is None:
                self._baseline_left = left_rad
                self._baseline_right = right_rad
            if self._relative_pose:
                self._pose_left = tuple(
                    value - base
                    for value, base in zip(left_rad, self._baseline_left)
                )
                self._pose_right = tuple(
                    value - base
                    for value, base in zip(right_rad, self._baseline_right)
                )
            else:
                self._pose_left = left_rad
                self._pose_right = right_rad
            self._snapshot = snapshot
            self._last_update = time.monotonic()
            self._connected = True
            self._status = "演示数据" if self._demo else "正在接收遥测"

    def _set_status(
        self,
        status: str,
        connected: bool,
        reset_baseline: bool = False,
        active_port: Optional[str] = None,
    ) -> None:
        with self._lock:
            self._status = status
            self._connected = connected
            if active_port is not None:
                self._active_port = active_port
            if reset_baseline:
                self._baseline_left = None
                self._baseline_right = None
                self._snapshot = None
                self._pose_left = (0.0,) * JOINT_COUNT
                self._pose_right = (0.0,) * JOINT_COUNT
                self._last_update = 0.0
                self._active_port = None


PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>外骨骼 7 自由度机械结构模型</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body {
      margin: 0; background: #10151c; color: #e9eef5;
      min-width: 320px;
    }
    #app { min-height: 100vh; }
    header {
      display: flex; align-items: flex-start; justify-content: space-between;
      gap: 16px; padding: 16px 20px 8px;
    }
    .header-copy { min-width: 0; }
    h1 { margin: 0; font-size: 20px; font-weight: 600; }
    .subtle { color: #9ca9b8; font-size: 13px; }
    #fullscreen {
      flex: 0 0 auto; min-height: 40px; padding: 8px 12px;
      border: 1px solid #435268; border-radius: 7px;
      background: #1d2937; color: #e9eef5; font: inherit; font-size: 13px;
      cursor: pointer;
    }
    #fullscreen:hover { background: #263649; }
    #fullscreen:focus-visible { outline: 2px solid #70a9ff; outline-offset: 2px; }
    #fullscreen:disabled { cursor: not-allowed; opacity: 0.55; }
    main {
      display: grid; grid-template-columns: minmax(0, 1fr) 330px;
      gap: 14px; padding: 0 20px 20px;
    }
    .stage, .panel {
      border: 1px solid #293443; border-radius: 10px; background: #151c25;
    }
    .stage { min-height: 520px; overflow: hidden; position: relative; }
    canvas { display: block; width: 100%; height: 620px; touch-action: none; cursor: grab; }
    canvas.dragging { cursor: grabbing; }
    .hint {
      position: absolute; left: 14px; bottom: 12px; color: #9ca9b8;
      font-size: 12px; pointer-events: none;
    }
    .panel { padding: 14px; overflow: auto; }
    .status {
      border-bottom: 1px solid #293443; padding-bottom: 12px; margin-bottom: 12px;
      line-height: 1.55; font-size: 13px;
    }
    .status strong { color: #ffbd61; }
    .status.ok strong { color: #65d79a; }
    .status.bad strong { color: #ff7777; }
    .readout { margin: 0 0 16px; }
    .readout h2 { margin: 0 0 6px; font-size: 14px; font-weight: 600; }
    table { border-collapse: collapse; width: 100%; font-size: 12px; }
    th, td { padding: 4px 3px; border-bottom: 1px solid #26313e; text-align: right; }
    th:first-child, td:first-child { text-align: left; }
    th { color: #9ca9b8; font-weight: 500; }
    .unknown { color: #ff7777; }
    #app:fullscreen {
      width: 100vw; height: 100vh; overflow: hidden; background: #10151c;
    }
    #app:fullscreen main {
      height: calc(100vh - 68px); grid-template-rows: minmax(0, 1fr);
    }
    #app:fullscreen .stage,
    #app:fullscreen .panel { min-height: 0; height: 100%; }
    #app:fullscreen canvas { height: 100%; }
    @media (max-width: 800px) {
      main { grid-template-columns: 1fr; padding: 0 12px 12px; }
      header { padding-left: 12px; padding-right: 12px; }
      header { flex-wrap: wrap; }
      .stage { min-height: 430px; }
      canvas { height: 480px; }
      .panel { overflow: visible; }
      #app:fullscreen { overflow: auto; }
      #app:fullscreen main { height: auto; }
      #app:fullscreen .stage,
      #app:fullscreen .panel { height: auto; }
      #app:fullscreen canvas { height: 480px; }
    }
  </style>
</head>
<body>
  <div id="app">
    <header>
      <div class="header-copy">
        <h1>外骨骼 7 自由度机械结构模型</h1>
        <div class="subtle">每个 slot 直接对应一个可见电机或转轴：肩部前后、肩部侧向、上臂旋转、肘部屈曲、前臂旋转、腕部两轴；鼠标拖动旋转视角，滚轮缩放，WASD 平移，空格上升，Shift 下降。</div>
      </div>
      <button id="fullscreen" type="button" aria-pressed="false">进入全屏</button>
    </header>
    <main>
      <section class="stage">
        <canvas id="scene" aria-label="外骨骼七自由度机械结构三维模型"></canvas>
        <div class="hint">鼠标拖动：旋转 · 滚轮：缩放 · WASD：前后左右 · Space：上升 · Shift：下降 · R：重置视角</div>
      </section>
      <aside class="panel">
        <div id="status" class="status" aria-live="polite">等待数据…</div>
        <section class="readout">
          <h2>左臂</h2>
          <table><thead><tr><th>slot</th><th>raw</th><th>rad</th><th>模型角</th></tr></thead>
          <tbody id="left-values"></tbody></table>
        </section>
        <section class="readout">
          <h2>右臂</h2>
          <table><thead><tr><th>slot</th><th>raw</th><th>rad</th><th>模型角</th></tr></thead>
          <tbody id="right-values"></tbody></table>
        </section>
        <div class="subtle">现场第一人称方向表作为正方向依据；查看器坐标为 +X 穿戴者右侧、+Y 面向方向、+Z 向上。模型按机械串联结构显示 7 个独立电机：slot 0/1 肩部两轴，slot 2 上臂中段旋转，slot 3 肘部屈曲，slot 4 小臂旋转，slot 5/6 腕部两轴。每个槽位直接使用弧度并叠加现场正方向符号；“模型角”列就是叠加符号后的角度，默认相对首帧，需要查看原始绝对弧度时启动参数加 <code>--absolute</code>。遥操作若只需要 6 自由度，应在上层另选一个腕部槽位映射，不在本诊断模型中删除第二个腕部电机。</div>
      </aside>
    </main>
  </div>
  <script>
    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    const app = document.getElementById('app');
    const fullscreenButton = document.getElementById('fullscreen');
    const state = { value: null };
    const camera = {
      yaw: Math.PI, pitch: -0.28, zoom: 1.0,
      position: {x:0, y:0, z:0.22}
    };
    const heldKeys = new Set();
    const movementKeys = new Set([
      'KeyW', 'KeyA', 'KeyS', 'KeyD', 'Space', 'ShiftLeft', 'ShiftRight'
    ]);
    let lastMoveTime = performance.now();

    function add(a, b) { return {x:a.x+b.x, y:a.y+b.y, z:a.z+b.z}; }
    function scale(a, k) { return {x:a.x*k, y:a.y*k, z:a.z*k}; }
    function norm(a) {
      const length = Math.hypot(a.x, a.y, a.z) || 1;
      return scale(a, 1 / length);
    }
    function clamp(value, low, high) { return Math.max(low, Math.min(high, value)); }
    function cross(a, b) {
      return {x:a.y*b.z-a.z*b.y, y:a.z*b.x-a.x*b.z, z:a.x*b.y-a.y*b.x};
    }
    function rotate(v, axis, angle) {
      const u = norm(axis), c = Math.cos(angle), s = Math.sin(angle);
      const dot = v.x*u.x + v.y*u.y + v.z*u.z;
      const crossValue = cross(u, v);
      return {
        x: v.x*c + crossValue.x*s + u.x*dot*(1-c),
        y: v.y*c + crossValue.y*s + u.y*dot*(1-c),
        z: v.z*c + crossValue.z*s + u.z*dot*(1-c)
      };
    }
    function project(point) {
      const cy = Math.cos(camera.yaw), sy = Math.sin(camera.yaw);
      const relative = {
        x: point.x - camera.position.x,
        y: point.y - camera.position.y,
        z: point.z - camera.position.z
      };
      const x1 = cy*relative.x - sy*relative.y;
      const depthAxis = sy*relative.x + cy*relative.y;
      const cp = Math.cos(camera.pitch), sp = Math.sin(camera.pitch);
      const y2 = cp*relative.z - sp*depthAxis;
      const depth = 4.5 + cp*depthAxis + sp*relative.z;
      const scaleValue = camera.zoom * 720 / Math.max(1.0, depth);
      return { x: canvas.clientWidth/2 + x1*scaleValue,
               y: canvas.clientHeight/2 - y2*scaleValue,
               depth };
    }
    function rotateFrame(frame, axis, angle) {
      frame.x = rotate(frame.x, axis, angle);
      frame.y = rotate(frame.y, axis, angle);
      frame.z = rotate(frame.z, axis, angle);
    }
    function copyVector(value) {
      return {x:value.x, y:value.y, z:value.z};
    }
    function copyFrame(frame) {
      return {x:copyVector(frame.x), y:copyVector(frame.y), z:copyVector(frame.z)};
    }
    function armPoints(side, values) {
      const lengths = { upper:0.37, forearm:0.35, hand:0.20 };
      // 世界坐标固定为：+X=穿戴者右，+Y=穿戴者前，+Z=向上。
      // 这是统一的人体右手基准：frame.x=身体右方，frame.y=身体后方，frame.z=手臂下方。
      // slot 2/4 的顺逆时针均按穿戴者低头俯视肘部的视野解释。
      const signs = side < 0 ? {
        shoulderFrontBack: -1,  // 左 slot 0：向后
        shoulderLateral: 1,     // 左 slot 1：向身体内侧
        upperArmRotation: -1,   // 左 slot 2：俯视逆时针
        elbowFrontBack: 1,      // 左 slot 3：小臂向面部/向上屈肘
        elbowRotation: 1,       // 左 slot 4：俯视顺时针
        wristFrontBack: -1,     // 左 slot 5：向后
        wristLateral: 1         // 左 slot 6：向身体中心
      } : {
        shoulderFrontBack: 1,   // 右 slot 0：向前（现场确认）
        shoulderLateral: 1,     // 右 slot 1：向身体外侧
        upperArmRotation: -1,   // 右 slot 2：俯视逆时针
        elbowFrontBack: 1,      // 右 slot 3：小臂向面部/向上屈肘
        elbowRotation: 1,       // 右 slot 4：俯视顺时针
        wristFrontBack: -1,     // 右 slot 5：向后
        wristLateral: 1         // 右 slot 6：向身体外侧
      };
      const frame = {
        x: {x:1, y:0, z:0},
        y: {x:0, y:-1, z:0},
        z: {x:0, y:0, z:-1}
      };
      const shoulder = {x:side*0.34, y:0, z:1.48};
      const motors = [];
      const segments = [];
      const value = slot => {
        const numeric = Number(values[slot]);
        return Number.isFinite(numeric) ? numeric : 0;
      };
      const addMotor = (slot, label, center, axis, sign) => {
        motors.push({
          slot, label, center:copyVector(center), axis:norm(axis),
          rawAngle:value(slot), mappedAngle:sign * value(slot), sign
        });
      };
      const apply = (axisName, slot, sign) => {
        rotateFrame(frame, frame[axisName], sign * value(slot));
      };
      // 机械串联顺序：肩部前后、肩部侧向、上臂中段旋转。
      addMotor(0, '肩前后', add(shoulder, scale(frame.y, 0.055)), frame.x, signs.shoulderFrontBack);
      apply('x', 0, signs.shoulderFrontBack);
      addMotor(1, '肩侧向', add(shoulder, scale(frame.x, 0.055)), frame.y, signs.shoulderLateral);
      apply('y', 1, signs.shoulderLateral);
      const upperDirection = norm(frame.z);
      const upperMiddle = add(shoulder, scale(upperDirection, lengths.upper * 0.5));
      addMotor(2, '上臂旋转', upperMiddle, frame.z, signs.upperArmRotation);
      apply('z', 2, signs.upperArmRotation);
      const elbow = add(shoulder, scale(norm(frame.z), lengths.upper));
      const upperFrame = copyFrame(frame);

      // 肘部屈曲后得到小臂，再在小臂上安装独立的旋转电机。
      addMotor(3, '肘部屈曲', add(elbow, scale(frame.y, 0.055)), frame.x, signs.elbowFrontBack);
      apply('x', 3, signs.elbowFrontBack);
      const wrist = add(elbow, scale(norm(frame.z), lengths.forearm));
      const forearmMiddle = add(elbow, scale(norm(frame.z), lengths.forearm * 0.5));
      addMotor(4, '小臂旋转', forearmMiddle, frame.z, signs.elbowRotation);
      apply('z', 4, signs.elbowRotation);
      const forearmFrame = copyFrame(frame);

      // 腕部两个电机共用同一个腕部位置，但各自保留独立的转轴。
      addMotor(5, '腕前后', add(wrist, scale(frame.y, 0.060)), frame.x, signs.wristFrontBack);
      apply('x', 5, signs.wristFrontBack);
      addMotor(6, '腕左右', add(wrist, scale(frame.x, 0.060)), frame.y, signs.wristLateral);
      apply('y', 6, signs.wristLateral);
      const hand = add(wrist, scale(norm(frame.z), lengths.hand));
      segments.push({
        start:copyVector(shoulder), end:copyVector(elbow),
        widthAxis:copyVector(upperFrame.x), thicknessAxis:copyVector(upperFrame.y)
      });
      segments.push({
        start:copyVector(elbow), end:copyVector(wrist),
        widthAxis:copyVector(forearmFrame.x), thicknessAxis:copyVector(forearmFrame.y)
      });
      return {
        points: [shoulder, elbow, wrist, hand],
        segments,
        motors,
        frame: copyFrame(frame)
      };
    }
    function drawLine(points, color, width, dash = []) {
      if (points.length < 2) return;
      const projected = points.map(project);
      ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width;
      ctx.setLineDash(dash); ctx.beginPath();
      ctx.moveTo(projected[0].x, projected[0].y);
      for (let index = 1; index < projected.length; index += 1) {
        ctx.lineTo(projected[index].x, projected[index].y);
      }
      ctx.stroke(); ctx.restore();
    }
    function drawPolygon(points, fill, stroke = null, width = 1) {
      const projected = points.map(project);
      if (projected.length < 3) return;
      ctx.save(); ctx.beginPath();
      ctx.moveTo(projected[0].x, projected[0].y);
      for (let index = 1; index < projected.length; index += 1) {
        ctx.lineTo(projected[index].x, projected[index].y);
      }
      ctx.closePath(); ctx.fillStyle = fill; ctx.fill();
      if (stroke) {
        ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.stroke();
      }
      ctx.restore();
    }
    function dot(a, b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
    function litColor(color, brightness) {
      const hex = color.replace('#', '');
      const red = parseInt(hex.slice(0, 2), 16);
      const green = parseInt(hex.slice(2, 4), 16);
      const blue = parseInt(hex.slice(4, 6), 16);
      const channel = value => Math.max(0, Math.min(255, Math.round(value * brightness)));
      return `rgb(${channel(red)},${channel(green)},${channel(blue)})`;
    }
    function addFace(faces, points, color, stroke = '#182331') {
      if (points.length < 3) return;
      const projected = points.map(project);
      const edgeA = add(points[1], scale(points[0], -1));
      const edgeB = add(points[2], scale(points[0], -1));
      const normal = norm(cross(edgeA, edgeB));
      const light = norm({x:-0.45, y:-0.70, z:1.0});
      const brightness = 0.56 + 0.44 * Math.abs(dot(normal, light));
      const depth = projected.reduce((sum, point) => sum + point.depth, 0) / projected.length;
      faces.push({projected, depth, fill:litColor(color, brightness), stroke});
    }
    function drawFaces(faces) {
      // Painter's algorithm: larger depth is farther from the camera.
      faces.sort((first, second) => second.depth - first.depth);
      for (const face of faces) {
        ctx.save(); ctx.beginPath();
        ctx.moveTo(face.projected[0].x, face.projected[0].y);
        for (let index = 1; index < face.projected.length; index += 1) {
          ctx.lineTo(face.projected[index].x, face.projected[index].y);
        }
        ctx.closePath(); ctx.fillStyle = face.fill; ctx.fill();
        if (face.stroke) {
          ctx.strokeStyle = face.stroke; ctx.lineWidth = 0.7; ctx.stroke();
        }
        ctx.restore();
      }
    }
    function addTaperedBox(faces, z0, z1, width0, depth0, width1, depth1, color) {
      const bottom = [
        {x:-width0,y:-depth0,z:z0}, {x:width0,y:-depth0,z:z0},
        {x:width0,y:depth0,z:z0}, {x:-width0,y:depth0,z:z0}
      ];
      const top = [
        {x:-width1,y:-depth1,z:z1}, {x:width1,y:-depth1,z:z1},
        {x:width1,y:depth1,z:z1}, {x:-width1,y:depth1,z:z1}
      ];
      addFace(faces, bottom.slice().reverse(), color);
      addFace(faces, top, color);
      for (let index = 0; index < 4; index += 1) {
        const next = (index + 1) % 4;
        addFace(faces, [bottom[index], bottom[next], top[next], top[index]], color);
      }
    }
    function addCylinder(faces, start, end, radius, color, sides = 8) {
      const axis = norm(add(end, scale(start, -1)));
      const reference = Math.abs(axis.z) < 0.85 ? {x:0,y:0,z:1} : {x:0,y:1,z:0};
      const basisU = norm(cross(axis, reference));
      const basisV = norm(cross(axis, basisU));
      const startRing = [], endRing = [];
      for (let index = 0; index < sides; index += 1) {
        const angle = 2 * Math.PI * index / sides;
        const radial = add(
          scale(basisU, Math.cos(angle) * radius),
          scale(basisV, Math.sin(angle) * radius)
        );
        startRing.push(add(start, radial));
        endRing.push(add(end, radial));
      }
      for (let index = 0; index < sides; index += 1) {
        const next = (index + 1) % sides;
        addFace(faces, [startRing[index], startRing[next], endRing[next], endRing[index]], color);
      }
      addFace(faces, startRing.slice().reverse(), color);
      addFace(faces, endRing, color);
    }
    function addBoxBetween(faces, start, end, widthAxis, thicknessAxis, halfWidth, halfThickness, color) {
      const width = norm(widthAxis), thickness = norm(thicknessAxis);
      const corner = (center, widthSign, thicknessSign) => add(
        add(center, scale(width, widthSign * halfWidth)),
        scale(thickness, thicknessSign * halfThickness)
      );
      const startRing = [
        corner(start, -1, -1), corner(start, 1, -1),
        corner(start, 1, 1), corner(start, -1, 1)
      ];
      const endRing = [
        corner(end, -1, -1), corner(end, 1, -1),
        corner(end, 1, 1), corner(end, -1, 1)
      ];
      addFace(faces, startRing.slice().reverse(), color);
      addFace(faces, endRing, color);
      for (let index = 0; index < 4; index += 1) {
        const next = (index + 1) % 4;
        addFace(faces, [startRing[index], startRing[next], endRing[next], endRing[index]], color);
      }
    }
    function addEllipsoidOriented(faces, center, longitudinal, width, thickness, radii, color, sides = 10, rings = 5) {
      const axisA = norm(longitudinal);
      const axisB = norm(width);
      const axisC = norm(thickness);
      const point = (latitude, longitude) => ({
        x:center.x
          + axisA.x * radii.x * Math.cos(latitude) * Math.cos(longitude)
          + axisB.x * radii.y * Math.cos(latitude) * Math.sin(longitude)
          + axisC.x * radii.z * Math.sin(latitude),
        y:center.y
          + axisA.y * radii.x * Math.cos(latitude) * Math.cos(longitude)
          + axisB.y * radii.y * Math.cos(latitude) * Math.sin(longitude)
          + axisC.y * radii.z * Math.sin(latitude),
        z:center.z
          + axisA.z * radii.x * Math.cos(latitude) * Math.cos(longitude)
          + axisB.z * radii.y * Math.cos(latitude) * Math.sin(longitude)
          + axisC.z * radii.z * Math.sin(latitude)
      });
      for (let ring = 0; ring < rings; ring += 1) {
        const latitude0 = -Math.PI/2 + Math.PI * ring / rings;
        const latitude1 = -Math.PI/2 + Math.PI * (ring + 1) / rings;
        for (let index = 0; index < sides; index += 1) {
          const longitude0 = 2 * Math.PI * index / sides;
          const longitude1 = 2 * Math.PI * (index + 1) / sides;
          addFace(faces, [
            point(latitude0, longitude0), point(latitude0, longitude1),
            point(latitude1, longitude1), point(latitude1, longitude0)
          ], color);
        }
      }
    }
    function addEllipsoid(faces, center, radii, color, sides = 10, rings = 5) {
      addEllipsoidOriented(
        faces, center, {x:1,y:0,z:0}, {x:0,y:1,z:0}, {x:0,y:0,z:1},
        radii, color, sides, rings
      );
    }
    function addSphere(faces, center, radius, color) {
      addEllipsoid(faces, center, {x:radius,y:radius,z:radius}, color, 10, 5);
    }
    function addCapsule(faces, start, end, radius, color) {
      addCylinder(faces, start, end, radius, color, 8);
      addSphere(faces, start, radius, color);
      addSphere(faces, end, radius, color);
    }
    function addMotorHousing(faces, motor, linkColor, accentColor) {
      const axis = norm(motor.axis);
      const halfLength = motor.slot === 2 || motor.slot === 4 ? 0.060 : 0.052;
      const radius = motor.slot === 3 ? 0.082 : 0.070;
      const bodyStart = add(motor.center, scale(axis, -halfLength));
      const bodyEnd = add(motor.center, scale(axis, halfLength));
      addCylinder(faces, bodyStart, bodyEnd, radius, '#aab7c4', 10);
      const collarHalf = Math.min(0.018, halfLength * 0.45);
      addCylinder(
        faces,
        add(motor.center, scale(axis, -collarHalf)),
        add(motor.center, scale(axis, collarHalf)),
        radius * 1.12,
        accentColor,
        10
      );
      addCylinder(
        faces,
        add(motor.center, scale(axis, halfLength * 0.82)),
        add(motor.center, scale(axis, halfLength * 1.04)),
        radius * 0.42,
        linkColor,
        8
      );
    }
    function addRobotHand(faces, wrist, hand, frame, color, side) {
      const handAxis = norm(add(hand, scale(wrist, -1)));
      const handWidth = norm(frame.x);
      const handThickness = norm(frame.y);
      const palmStart = add(wrist, scale(handAxis, 0.018));
      const palmEnd = add(wrist, scale(handAxis, 0.135));
      addBoxBetween(faces, palmStart, palmEnd, handWidth, handThickness, 0.082, 0.045, color);
      for (let index = -1; index <= 1; index += 1) {
        const fingerBase = add(palmEnd, scale(handWidth, index * 0.042));
        const fingerTip = add(fingerBase, scale(handAxis, 0.115 - Math.abs(index) * 0.012));
        addBoxBetween(faces, fingerBase, fingerTip, handWidth, handThickness, 0.021, 0.017, color);
        addSphere(faces, fingerTip, 0.022, color);
      }
      const thumbBase = add(wrist, scale(handAxis, 0.055));
      const thumbTip = add(
        add(thumbBase, scale(handWidth, -side * 0.105)),
        scale(handAxis, 0.065)
      );
      addBoxBetween(faces, thumbBase, thumbTip, handWidth, handThickness, 0.024, 0.020, color);
      addSphere(faces, thumbTip, 0.025, color);
    }
    function addRobotArmModel(faces, arm, side) {
      const linkColor = side < 0 ? '#2f91bd' : '#c47d32';
      const accentColor = side < 0 ? '#56c2ff' : '#ffb35a';
      for (const segment of arm.segments) {
        addBoxBetween(
          faces, segment.start, segment.end,
          segment.widthAxis, segment.thicknessAxis,
          0.060, 0.050, linkColor
        );
      }
      addRobotHand(faces, arm.points[2], arm.points[3], arm.frame, linkColor, side);
      for (const motor of arm.motors) {
        addMotorHousing(faces, motor, linkColor, accentColor);
      }
      addSphere(faces, arm.points[0], 0.092, accentColor);
      addSphere(faces, arm.points[1], 0.078, accentColor);
      addSphere(faces, arm.points[2], 0.070, accentColor);
    }
    function addRobotBody(faces, left, right) {
      // A blocky robot torso provides a stable reference for the seven actuator axes.
      addTaperedBox(faces, 0.87, 1.50, 0.22, 0.13, 0.29, 0.16, '#7e91a7');
      addTaperedBox(faces, 0.69, 0.93, 0.23, 0.14, 0.21, 0.13, '#667b93');
      addBoxBetween(
        faces, {x:-0.52,y:0,z:1.48}, {x:0.52,y:0,z:1.48},
        {x:0,y:1,z:0}, {x:0,y:0,z:1}, 0.085, 0.085, '#60758c'
      );
      addCylinder(faces, {x:0,y:0,z:1.50}, {x:0,y:0,z:1.66}, 0.085, '#93a5b8', 10);
      addEllipsoid(faces, {x:0,y:0,z:1.81}, {x:0.135,y:0.12,z:0.16}, '#a9b7c7', 10, 6);

      const hipLeft = {x:-0.14,y:0,z:0.83};
      const hipRight = {x:0.14,y:0,z:0.83};
      const kneeLeft = {x:-0.15,y:0,z:0.45};
      const kneeRight = {x:0.15,y:0,z:0.45};
      const ankleLeft = {x:-0.15,y:0,z:0.10};
      const ankleRight = {x:0.15,y:0,z:0.10};
      addCapsule(faces, hipLeft, kneeLeft, 0.095, '#8798aa');
      addCapsule(faces, hipRight, kneeRight, 0.095, '#8798aa');
      addCapsule(faces, kneeLeft, ankleLeft, 0.075, '#718498');
      addCapsule(faces, kneeRight, ankleRight, 0.075, '#718498');
      addSphere(faces, kneeLeft, 0.085, '#a9b7c7');
      addSphere(faces, kneeRight, 0.085, '#a9b7c7');
      addCapsule(faces, ankleLeft, {x:-0.15,y:0.16,z:0.075}, 0.075, '#53687f');
      addCapsule(faces, ankleRight, {x:0.15,y:0.16,z:0.075}, 0.075, '#53687f');

      addRobotArmModel(faces, left, -1);
      addRobotArmModel(faces, right, 1);
    }
    function drawLabel(point, label) {
      const projected = project(point);
      ctx.save(); ctx.fillStyle = '#dce5ef'; ctx.font = '12px system-ui, sans-serif';
      ctx.fillText(label, projected.x + 7, projected.y - 7); ctx.restore();
    }
    function drawMotorMarker(motor, label, color) {
      const axis = norm(motor.axis);
      const span = 0.105;
      drawLine(
        [add(motor.center, scale(axis, -span)), add(motor.center, scale(axis, span))],
        color, 1.5, [4, 3]
      );
      drawNode(motor.center, color, 0.020, label);
    }
    function drawFloor() {
      const size = 4.5;
      const corners = [
        {x:-size, y:-size, z:0}, {x:size, y:-size, z:0},
        {x:size, y:size, z:0}, {x:-size, y:size, z:0}
      ];
      drawPolygon(corners, '#244c31', '#5b9b68', 2);
      for (let coordinate = -size; coordinate <= size; coordinate += 0.5) {
        drawLine(
          [{x:coordinate, y:-size, z:0.003}, {x:coordinate, y:size, z:0.003}],
          '#3d7049', 1
        );
        drawLine(
          [{x:-size, y:coordinate, z:0.003}, {x:size, y:coordinate, z:0.003}],
          '#3d7049', 1
        );
      }
    }
    function drawNode(point, color, radius, label = '') {
      const p = project(point);
      const size = Math.max(3.0, radius * 720 / Math.max(1.0, p.depth));
      ctx.save(); ctx.fillStyle = color; ctx.beginPath();
      ctx.arc(p.x, p.y, size, 0, Math.PI*2); ctx.fill();
      if (label) {
        ctx.fillStyle = '#dce5ef'; ctx.font = '12px system-ui, sans-serif';
        ctx.fillText(label, p.x + size + 3, p.y - size - 2);
      }
      ctx.restore();
    }
    function drawFacingMarker() {
      const base = {x:0, y:0.03, z:1.30};
      const tip = {x:0, y:0.30, z:1.30};
      const wingLeft = {x:-0.06, y:0.22, z:1.30};
      const wingRight = {x:0.06, y:0.22, z:1.30};
      drawLine([base, tip], '#f2d36b', 3);
      drawLine([wingLeft, tip, wingRight], '#f2d36b', 3);
      drawNode(tip, '#f2d36b', 0.030, '前');
    }
    function drawAxes() {
      const origin = {x:-0.95, y:-0.65, z:0.12};
      const axes = [
        [add(origin, {x:0.25,y:0,z:0}), '#ff7777', '+X 右'],
        [add(origin, {x:0,y:0.25,z:0}), '#65d79a', '+Y 前'],
        [add(origin, {x:0,y:0,z:0.25}), '#70a9ff', '+Z 上']
      ];
      for (const [end, color, label] of axes) {
        drawLine([origin, end], color, 2);
        drawNode(end, color, 0.018, label);
      }
    }
    function drawScene() {
      const width = canvas.clientWidth, height = canvas.clientHeight;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = '#151c25'; ctx.fillRect(0, 0, width, height);
      const current = state.value || {
        left_pose_rad: [0,0,0,0,0,0,0,0],
        right_pose_rad: [0,0,0,0,0,0,0,0]
      };
      const leftArm = armPoints(-1, current.left_pose_rad || []);
      const rightArm = armPoints(1, current.right_pose_rad || []);
      const left = leftArm.points;
      const right = rightArm.points;
      drawFloor();
      const faces = [];
      addRobotBody(faces, leftArm, rightArm);
      drawFaces(faces);
      drawFacingMarker();
      for (let index = 0; index < left.length; index += 1) {
        const leftLabel = ['L肩', 'L肘', 'L腕', 'L手'][index];
        const rightLabel = ['R肩', 'R肘', 'R腕', 'R手'][index];
        drawLabel(left[index], leftLabel);
        drawLabel(right[index], rightLabel);
      }
      for (const motor of leftArm.motors) {
        drawMotorMarker(motor, `L${motor.slot}`, '#8bdcff');
      }
      for (const motor of rightArm.motors) {
        drawMotorMarker(motor, `R${motor.slot}`, '#ffd08a');
      }
      const leftGhost = add(left[left.length-1], {x:-0.09, y:0, z:0});
      const rightGhost = add(right[right.length-1], {x:0.09, y:0, z:0});
      drawLine([left[left.length-1], leftGhost], '#ff7777', 2, [5, 4]);
      drawLine([right[right.length-1], rightGhost], '#ff7777', 2, [5, 4]);
      drawNode(leftGhost, '#ff7777', 0.045, 'L7*');
      drawNode(rightGhost, '#ff7777', 0.045, 'R7*');
      drawAxes();
    }
    function fmt(value) { return Number(value || 0).toFixed(3); }
    function modelAngles(side, pose) {
      const signs = side < 0 ? [-1, 1, -1, 1, 1, -1, 1] : [1, 1, -1, 1, 1, -1, 1];
      return pose.map((value, index) => (index < signs.length ? Number(value || 0) * signs[index] : 0));
    }
    function renderArm(target, raw, rad, pose, side) {
      const mapped = modelAngles(side, pose);
      target.innerHTML = raw.map((value, index) => {
        const unknown = index === 7 ? ' class="unknown"' : '';
        return `<tr${unknown}><td>slot ${index}</td><td>${value}</td><td>${fmt(rad[index])}</td><td>${fmt(mapped[index])}</td></tr>`;
      }).join('');
    }
    function updatePanel(value) {
      const status = document.getElementById('status');
      const stateText = value.fresh ? 'OK · 数据新鲜' : (value.connected ? 'WAIT · 等待/超时' : 'ERROR · 未连接');
      status.className = `status ${value.fresh ? 'ok' : 'bad'}`;
      const age = value.age_ms == null ? '—' : `${value.age_ms.toFixed(0)} ms`;
      const port = value.port || (value.source === 'demo' ? '离线演示' : '等待自动发现');
      status.innerHTML = `<strong>${stateText}</strong><br>${value.status}<br>绑定：VID:PID=${value.usb_id} · ${port}<br>来源：${value.source} · frame=${value.frame_size || '—'} · format=${value.format_version || '—'} · age=${age}`;
      renderArm(document.getElementById('left-values'), value.left_raw, value.left_rad, value.left_pose_rad, -1);
      renderArm(document.getElementById('right-values'), value.right_raw, value.right_rad, value.right_pose_rad, 1);
    }
    function resizeCanvas() {
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.floor(canvas.clientWidth * ratio);
      canvas.height = Math.floor(canvas.clientHeight * ratio);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      drawScene();
    }
    let dragState = null;
    canvas.addEventListener('pointerdown', event => {
      if (event.button !== 0) return;
      event.preventDefault();
      dragState = {pointerId:event.pointerId, x:event.clientX, y:event.clientY};
      canvas.classList.add('dragging');
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener('pointermove', event => {
      if (!dragState || event.pointerId !== dragState.pointerId) return;
      event.preventDefault();
      const dx = event.clientX - dragState.x;
      const dy = event.clientY - dragState.y;
      dragState.x = event.clientX;
      dragState.y = event.clientY;
      camera.yaw += dx * 0.008;
      camera.pitch = clamp(camera.pitch + dy * 0.008, -1.30, 1.30);
      drawScene();
    });
    function endDrag(event) {
      if (!dragState || event.pointerId !== dragState.pointerId) return;
      dragState = null;
      canvas.classList.remove('dragging');
    }
    canvas.addEventListener('pointerup', endDrag);
    canvas.addEventListener('pointercancel', endDrag);
    canvas.addEventListener('lostpointercapture', () => {
      dragState = null;
      canvas.classList.remove('dragging');
    });
    canvas.addEventListener('wheel', event => {
      event.preventDefault(); camera.zoom = Math.max(0.55, Math.min(2.0, camera.zoom * Math.exp(-event.deltaY * 0.001))); drawScene();
    }, {passive:false});
    function resetCamera() {
      camera.yaw = Math.PI; camera.pitch = -0.28; camera.zoom = 1.0;
      camera.position = {x:0, y:0, z:0.22};
      drawScene();
    }
    function moveCamera(deltaSeconds) {
      let forward = 0;
      let strafe = 0;
      let vertical = 0;
      if (heldKeys.has('KeyW')) forward += 1;
      if (heldKeys.has('KeyS')) forward -= 1;
      if (heldKeys.has('KeyD')) strafe += 1;
      if (heldKeys.has('KeyA')) strafe -= 1;
      if (heldKeys.has('Space')) vertical += 1;
      if (heldKeys.has('ShiftLeft') || heldKeys.has('ShiftRight')) vertical -= 1;
      if (!forward && !strafe && !vertical) return false;

      const horizontalLength = Math.hypot(forward, strafe) || 1;
      forward /= horizontalLength;
      strafe /= horizontalLength;
      const distance = 1.15 * deltaSeconds;
      const forwardVector = {x:Math.sin(camera.yaw), y:Math.cos(camera.yaw)};
      const rightVector = {x:Math.cos(camera.yaw), y:-Math.sin(camera.yaw)};
      camera.position.x += distance * (forward*forwardVector.x + strafe*rightVector.x);
      camera.position.y += distance * (forward*forwardVector.y + strafe*rightVector.y);
      camera.position.z += distance * vertical;
      return true;
    }
    function animateMovement(now) {
      const deltaSeconds = Math.min(0.05, Math.max(0, (now - lastMoveTime) / 1000));
      lastMoveTime = now;
      if (moveCamera(deltaSeconds)) drawScene();
      window.requestAnimationFrame(animateMovement);
    }
    window.addEventListener('keydown', event => {
      if (movementKeys.has(event.code)) {
        event.preventDefault();
        heldKeys.add(event.code);
        return;
      }
      if (event.code === 'KeyR' && !event.repeat) {
        event.preventDefault();
        resetCamera();
      }
    });
    window.addEventListener('keyup', event => { heldKeys.delete(event.code); });
    window.addEventListener('blur', () => { heldKeys.clear(); });
    window.addEventListener('resize', resizeCanvas);
    function updateFullscreenButton() {
      const active = document.fullscreenElement === app;
      fullscreenButton.textContent = active ? '退出全屏' : '进入全屏';
      fullscreenButton.setAttribute('aria-pressed', String(active));
    }
    fullscreenButton.addEventListener('click', async () => {
      try {
        if (document.fullscreenElement) {
          await document.exitFullscreen();
        } else if (app.requestFullscreen) {
          await app.requestFullscreen();
        } else {
          fullscreenButton.disabled = true;
          fullscreenButton.textContent = '当前浏览器不支持全屏';
        }
      } catch (error) {
        fullscreenButton.textContent = '全屏不可用';
      }
      updateFullscreenButton();
      resizeCanvas();
    });
    document.addEventListener('fullscreenchange', () => {
      updateFullscreenButton();
      resizeCanvas();
    });
    resizeCanvas();
    window.requestAnimationFrame(animateMovement);
    async function refresh() {
      try {
        const response = await fetch(`/state?ts=${Date.now()}`, {cache:'no-store'});
        state.value = await response.json(); updatePanel(state.value); drawScene();
      } catch (error) {
        document.getElementById('status').innerHTML = '<strong>ERROR · 页面无法读取状态</strong>';
      }
      window.setTimeout(refresh, 80);
    }
    refresh();
  </script>
</body>
</html>
"""


def make_handler(source: TelemetrySource) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - required HTTP method name
            if self.path.split("?", 1)[0] == "/state":
                body = json.dumps(
                    source.state(),
                    ensure_ascii=False,
                    separators=(",", ":"),
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path.split("?", 1)[0] == "/":
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            self.send_error(404)

        def log_message(self, format_string: str, *args: object) -> None:
            return

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="用 3D 七自由度机械结构观察外骨骼左右臂 8 个编码器槽位"
    )
    parser.add_argument(
        "--port",
        default=None,
        help="显式指定串口路径；默认按 USB VID:PID 自动发现",
    )
    parser.add_argument(
        "--vid",
        type=parse_usb_id,
        default=0x0483,
        help="USB VID，默认 0x0483",
    )
    parser.add_argument(
        "--pid",
        type=parse_usb_id,
        default=0x5740,
        help="USB PID，默认 0x5740",
    )
    parser.add_argument("--baudrate", type=int, default=2_000_000)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="不打开串口，使用离线运动数据演示",
    )
    parser.add_argument(
        "--absolute",
        action="store_true",
        help="使用绝对编码器弧度作为近似姿态，默认显示相对首帧变化量",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.30,
        help="页面判定数据超时的秒数，默认 0.30",
    )
    parser.add_argument(
        "--http-host",
        default="127.0.0.1",
        help="本地页面监听地址，默认 127.0.0.1",
    )
    parser.add_argument(
        "--http-port",
        type=int,
        default=8765,
        help="本地页面监听端口，默认 8765",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="只启动页面服务，不自动打开浏览器",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.baudrate <= 0:
        raise SystemExit("--baudrate 必须大于 0")
    if args.stale_timeout <= 0:
        raise SystemExit("--stale-timeout 必须大于 0")
    if not 0 <= args.http_port <= 65535:
        raise SystemExit("--http-port 必须在 0..65535 范围内")

    source = TelemetrySource(
        port=args.port,
        vid=args.vid,
        pid=args.pid,
        baudrate=args.baudrate,
        demo=args.demo,
        relative_pose=not args.absolute,
        stale_timeout=args.stale_timeout,
    )
    source.start()
    try:
        server = ThreadingHTTPServer(
            (args.http_host, args.http_port),
            make_handler(source),
        )
    except Exception:
        source.stop()
        raise

    display_host = "127.0.0.1" if args.http_host in ("0.0.0.0", "::") else args.http_host
    url = f"http://{display_host}:{server.server_port}/"
    print(f"3D viewer: {url}")
    if args.port:
        print(f"串口绑定：显式路径 {args.port}")
    else:
        print(f"串口绑定：USB VID:PID={format_usb_id(args.vid, args.pid)}")
    print("按 Ctrl-C 停止；鼠标拖动旋转视角，滚轮缩放，WASD 平移。")
    print("空格上升，Shift 下降，按 R 重置视角和位置；页面右上角可进入全屏。")
    if not args.no_browser:
        webbrowser.open(url)

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        source.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
