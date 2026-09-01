#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Qnbot Python SDK。

修订版本：V1.2
修订日期：20260306
作者：杭州启能机器人有限公司

使用说明（简要）：
- 安装依赖：pip install pyserial
- 作为库使用：
  - from qnbot_sdk import QnbotClient
  - with QnbotClient(port=\"/dev/tty.usbmodemXXXX\") as client: ...
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, Union
import queue
import struct
import threading
import time

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover - 仅做解析时可不安装依赖
    serial = None

# 协议常量
# -----------------------------

# legacy字节流：0xAA ... 0x55，XOR 校验
LEGACY_SOF = 0xAA
LEGACY_EOF = 0x55
LEGACY_PAYLOAD_LEN_BASE = 48
LEGACY_PAYLOAD_LEN_TORSO_IMU_QUAT = 88
LEGACY_PAYLOAD_LEN_TORSO_EXTRA_IMU_QUAT = 128
LEGACY_PAYLOAD_LENGTHS = (
    LEGACY_PAYLOAD_LEN_BASE,
    LEGACY_PAYLOAD_LEN_TORSO_IMU_QUAT,
    LEGACY_PAYLOAD_LEN_TORSO_EXTRA_IMU_QUAT,
)
LEGACY_FRAME_LENGTHS = tuple(length + 3 for length in LEGACY_PAYLOAD_LENGTHS)  # 帧头 + 载荷 + 校验 + 帧尾

# QnTP 协议
QNTP_SOF = 0xA5
QNTP_VERSION = 0x01
QNTP_HEADER_SIZE = 8  # SOF + Version + MsgType + MsgClass + MsgId + Seq + Len(2)
QNTP_CRC_SIZE = 2
QNTP_MAX_PAYLOAD = 4096

# QnTP 消息类型
QNTP_MSG_REQUEST = 0x01
QNTP_MSG_RESPONSE = 0x02
QNTP_MSG_NOTIFY = 0x03

# MsgClass（功能域）
MSGCLASS_SYSTEM = 0x01
MSGCLASS_DEVICE_PROFILE = 0x02
MSGCLASS_HAPTICS = 0x21
MSGCLASS_TELEMETRY = 0x10
MSGCLASS_ENCODER = 0x81
MSGCLASS_HANDSET = 0x82
MSGCLASS_IMU = 0x83
MSGCLASS_WIRELESS = 0x84

# MsgId
MSGID_SYSTEM_GET_VERSION = 0x01
MSGID_SYSTEM_GET_CAPS = 0x02
MSGID_SYSTEM_STATUS = 0x03
MSGID_SYSTEM_ENTER_DFU = 0x80
BATTERY_PERCENT_CHARGING = 0xFF

MSGID_DEVICE_PROFILE_GET_TOPOLOGY = 0x01
MSGID_DEVICE_PROFILE_GET_DATA_PROFILES = 0x02

MSGID_TELEMETRY_SET_STREAM_RUNTIME_CONFIG = 0x20
MSGID_TELEMETRY_HIGH_RATE_SNAPSHOT = 0x80


MSGID_ENCODER_GET_INFO = 0x01
MSGID_ENCODER_GET_INFO_LIST = 0x02
MSGID_ENCODER_GET_ZERO = 0x10
MSGID_ENCODER_SET_ZERO_HERE = 0x20
MSGID_ENCODER_SET_ZERO_VALUE = 0x21

MSGID_HAND_GET_INFO = 0x01
MSGID_HAND_GET_INFO_LIST = 0x02
MSGID_HAND_GET_CALIB_PARAMS = 0x10
MSGID_HAND_CALIB_START = 0x20
MSGID_HAND_CALIB_COMMIT = 0x21
MSGID_HAND_CALIB_FINISH = 0x22
MSGID_HAND_SET_CALIB_OUTPUT = 0x30
MSGID_HAND_SET_CALIB_PARAM = 0x31

MSGID_IMU_GET_INFO = 0x01
MSGID_IMU_GET_INFO_LIST = 0x02
MSGID_IMU_MAG_CALIBRATE = 0x10
MSGID_IMU_SET_OUTPUT_ENABLE = 0x20
MSGID_IMU_GET_OUTPUT_ENABLE = 0x21

MSGID_WIRELESS_GET_STATUS_INFO = 0x03
MSGID_WIRELESS_PAIR_START = 0x10
MSGID_WIRELESS_PAIR_CANCEL = 0x11
MSGID_WIRELESS_RESET = 0x12
MSGID_WIRELESS_ENTER_CONFIG = 0x13
MSGID_WIRELESS_EXIT_CONFIG = 0x14
MSGID_WIRELESS_SET_STREAM_CONFIG = 0x20
MSGID_WIRELESS_SET_PUSH_FREQ = 0x21
MSGID_WIRELESS_SET_PAIR_RESULT_INFO = 0x22

MSGID_HAPTICS_SET_OUTPUT = 0x01
MSGID_HAPTICS_DRV_GET_CAL_STATUS = 0x10
MSGID_HAPTICS_DRV_CALIBRATE = 0x11
MSGID_HAPTICS_VIBRATE_PLAY = 0x20
MSGID_HAPTICS_VIBRATE_STOP = 0x21
MSGID_HAPTICS_VIBRATE_REALTIME = 0x22
MSGID_HAPTICS_SET_ENABLE = 0x30
MSGID_HAPTICS_SET_MODE = 0x31
MSGID_HAPTICS_SET_PRESSURE = 0x32
MSGID_HAPTICS_SET_TIMEOUT = 0x33
MSGID_HAPTICS_GET_INTENSITY = 0x40

# 通用状态码
STATUS_OK = 0x00
STATUS_BAD_PARAM = 0x01
STATUS_BUSY = 0x02
STATUS_NOT_READY = 0x03
STATUS_UNSUPPORTED = 0x04

# Telemetry 常量
TELEMETRY_STREAM_ID_HIGH_RATE_SNAPSHOT = 0x00
TELEMETRY_STREAM_ID_LEGACY_PROTOCOL = 0xF1

# Hand 常量
HAND_CALIB_POINT_JX_CENTER = 1
HAND_CALIB_POINT_JX_MAX = 2
HAND_CALIB_POINT_JX_MIN = 3
HAND_CALIB_POINT_JY_CENTER = 4
HAND_CALIB_POINT_JY_MAX = 5
HAND_CALIB_POINT_JY_MIN = 6
HAND_CALIB_POINT_TRIG_START = 7
HAND_CALIB_POINT_TRIG_MAX = 8

# Wireless 常量
WIRELESS_DEVICE_NAME_LEN = 32
WIRELESS_PAIR_RESULT_NONE = 0
WIRELESS_PAIR_RESULT_RUNNING = 1
WIRELESS_PAIR_RESULT_OK = 2
WIRELESS_PAIR_RESULT_TIMEOUT = 3
WIRELESS_PAIR_RESULT_CANCELED = 4
WIRELESS_PAIR_RESULT_ERROR = 5

WIRELESS_PAIR_STEP_IDLE = 0
WIRELESS_PAIR_STEP_PREPARE = 1
WIRELESS_PAIR_STEP_SET_NAME = 2
WIRELESS_PAIR_STEP_SET_ROLE = 3
WIRELESS_PAIR_STEP_REPREPARE = 4
WIRELESS_PAIR_STEP_SEND_CMD = 5
WIRELESS_PAIR_STEP_WAIT_PEER = 6
WIRELESS_PAIR_STEP_FINISHING = 7

WIRELESS_ROLE_MASTER = 0
WIRELESS_ROLE_SLAVE = 1
WIRELESS_ROLE_DIRECT_PAIR = 2

# 编码器单位换算
ENCODER_TO_RADIAN_RATIO = 2.0 * 3.141592653589793 / 16384.0


# -----------------------------
# CRC16-CCITT（0x1021, 初值 0xFFFF）
# -----------------------------

_CRC16_TABLE: List[int] = []

def _init_crc16_table() -> None:
    """
    初始化 CRC16 查找表（模块级只需调用一次）。
    :return: None
    """
    poly = 0x1021
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        _CRC16_TABLE.append(crc)

_init_crc16_table()


def crc16_ccitt(data: bytes) -> int:
    """
    计算 CRC16-CCITT 校验值。
    :param data: 参与计算的字节序列
    :return: CRC16 校验值（0-65535）
    """
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) ^ _CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


# -----------------------------
# 数据结构
# -----------------------------

@dataclass
class TelemetrySnapshot:
    """
    遥测快照数据（legacy）。
    """
    protocol: str  # "legacy"
    timestamp: float
    joystick_left: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    joystick_right: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    arm_joint_left: List[int] = field(default_factory=lambda: [0] * 8)
    arm_joint_right: List[int] = field(default_factory=lambda: [0] * 8)
    arm_joint_left_rad: List[float] = field(default_factory=lambda: [0.0] * 8)
    arm_joint_right_rad: List[float] = field(default_factory=lambda: [0.0] * 8)
    torso_acc: List[float] = field(default_factory=lambda: [0.0] * 3)
    torso_gyro: List[float] = field(default_factory=lambda: [0.0] * 3)
    torso_quat: List[float] = field(default_factory=lambda: [0.0] * 4)
    extra_acc: List[float] = field(default_factory=lambda: [0.0] * 3)
    extra_gyro: List[float] = field(default_factory=lambda: [0.0] * 3)
    extra_quat: List[float] = field(default_factory=lambda: [0.0] * 4)
    format_version: int = 1


@dataclass
class QnTPFrame:
    """
    QnTP 帧的解析结果。
    """
    version: int
    msg_type: int
    msg_class: int
    msg_id: int
    seq: int
    payload: bytes
    raw: bytes


@dataclass
class SystemVersionInfo:
    """
    System.GetVersion 响应信息。
    """
    status_code: int
    proto_major: int
    proto_minor: int
    device_type: int
    platform: int
    revision: int
    feature: int
    build: int
    battery_mv: int = 0
    battery_percent: int = 0
    wireless_link_state: int = 0
    serial: bytes = b""
    device_name: str = ""


@dataclass
class SystemCapabilities:
    """
    System.GetCapabilities 响应信息。
    """
    status_code: int
    supported_class: bytes  # 32 字节位图


@dataclass
class SystemStatusInfo:
    """
    System.Status 响应信息。
    """
    status_code: int
    battery_mv: int
    battery_percent: int
    wireless_link_state: int
    serial: bytes = b""
    device_name: str = ""


@dataclass
class DeviceProfileTopology:
    """
    DeviceProfile.GetTopology 响应信息。
    """
    status_code: int
    encoder_count: int
    encoder_total_slots: int
    handset_count: int
    handset_total_slots: int
    imu_count: int
    imu_total_slots: int


@dataclass
class DeviceProfileDataProfiles:
    """
    DeviceProfile.GetDataProfiles 响应信息。
    当前固件通常返回 UNSUPPORTED，仍保留原始 payload 以便后续兼容。
    """
    status_code: int
    raw_payload: bytes = b""


@dataclass
class TelemetryStreamRuntimeConfigResult:
    """
    Telemetry.SetStreamRuntimeConfig 响应信息。
    """
    status_code: int
    stream_id: int


@dataclass
class EncoderInfo:
    """
    Encoder.GetInfo 响应信息。
    """
    status_code: int
    channel_index: int
    encoder_id: int
    hw_version: int
    fw_version: int
    serial: bytes


@dataclass
class EncoderInfoEntry:
    """
    编码器列表条目。
    """
    channel_index: int
    encoder_id: int
    hw_version: int
    fw_version: int
    serial: bytes


@dataclass
class EncoderInfoList:
    """
    Encoder.GetInfoList 响应信息。
    """
    status_code: int
    entries: List[EncoderInfoEntry]


@dataclass
class EncoderZeroValue:
    """
    Encoder.GetZeroValue 响应信息。
    """
    status_code: int
    channel_index: int
    zero_value: int


@dataclass
class EncoderSetZeroHereResult:
    """
    Encoder.SetZeroHere 响应信息。
    """
    status_code: int
    channel_index: int
    zero_offset: int
    angle_raw: Optional[int] = None


@dataclass
class EncoderSetZeroValueResult:
    """
    Encoder.SetZeroValue 响应信息。
    """
    status_code: int
    channel_index: int


@dataclass
class HandInfo:
    """
    Hand.GetInfo 响应信息。
    """
    status_code: int
    handset_index: int
    hw_version: int
    fw_version: int
    serial: bytes


@dataclass
class HandInfoEntry:
    """
    手柄列表条目。
    """
    handset_index: int
    hw_version: int
    fw_version: int
    serial: bytes


@dataclass
class HandInfoList:
    """
    Hand.GetInfoList 响应信息。
    """
    status_code: int
    entries: List[HandInfoEntry]


@dataclass
class HandCalibParams:
    """
    Hand.GetCalibParams 响应信息。
    """
    status_code: int
    handset_index: int
    calib_output_enabled: int
    jx_center: int
    jx_max: int
    jx_min: int
    jy_center: int
    jy_max: int
    jy_min: int
    trig_start: int
    trig_max: int


@dataclass
class HandCommandResult:
    """
    Hand 动作类命令通用响应。
    """
    status_code: int
    handset_index: int


@dataclass
class HandCalibCommitResult:
    """
    Hand.CalibCommit 响应信息。
    """
    status_code: int
    handset_index: int
    calib_point: int


@dataclass
class HandSetCalibOutputResult:
    """
    Hand.SetCalibOutput 响应信息。
    """
    status_code: int
    handset_index: int
    enable: int


@dataclass
class HandSetCalibParamResult:
    """
    Hand.SetCalibParam 响应信息。
    """
    status_code: int
    handset_index: int
    calib_point: int


@dataclass
class ImuInfo:
    """
    Imu.GetInfo 响应信息。
    """
    status_code: int
    imu_index: int
    version: int


@dataclass
class ImuInfoEntry:
    """
    IMU 列表条目。
    """
    imu_index: int
    version: int


@dataclass
class ImuInfoList:
    """
    Imu.GetInfoList 响应信息。
    """
    status_code: int
    entries: List[ImuInfoEntry]


@dataclass
class ImuMagCalibrateResult:
    """
    Imu.MagCalibrate 响应信息。
    """
    status_code: int
    action_echo: int
    imu_index: int
    state: int
    error_code: int


@dataclass
class ImuOutputEnableResult:
    """
    Imu.SetOutputEnable / Imu.GetOutputEnable 响应信息。
    """
    status_code: int
    enable: int
    persisted: int
    capability_mask: int
    error_code: int
    reserved: int = 0


@dataclass
class WirelessStatusInfo:
    """
    Wireless.GetStatusInfo 响应信息。
    """
    status_code: int
    pairing_busy: int
    pair_step: int
    pair_result: int
    in_config_mode: int
    link_state: int
    passthrough_enabled: int
    push_freq_option: int
    local_device_name: str = ""
    paired_device_name: str = ""


@dataclass
class WirelessCommandResult:
    """
    Wireless 动作/设置类命令的通用响应。
    """
    status_code: int


@dataclass
class WirelessPairWaitResult:
    """
    Wireless 配对等待结果。
    """
    accepted_status: int
    final_status: WirelessStatusInfo
    elapsed_s: float


@dataclass
class HapticsOutputResult:
    """
    Haptics.SetOutput 响应信息。
    """
    status_code: int
    channel_id: int


@dataclass
class HapticsDrvCalStatus:
    """
    Haptics.DrvGetCalStatus 响应信息。
    """
    status_code: int
    handset_index: int
    calibrated: int


@dataclass
class HapticsDrvCalibrateResult:
    """
    Haptics.DrvCalibrate 响应信息。
    """
    status_code: int
    handset_index: int
    result: int


@dataclass
class HapticsVibratePlayResult:
    """
    Haptics.VibratePlay 响应信息。
    """
    status_code: int
    handset_index: int
    effect_id: int


@dataclass
class HapticsVibrateStopResult:
    """
    Haptics.VibrateStop 响应信息。
    """
    status_code: int
    handset_index: int


@dataclass
class HapticsVibrateRealtimeResult:
    """
    Haptics.VibrateRealtime 响应信息。
    """
    status_code: int
    handset_index: int
    amplitude: int


@dataclass
class HapticsEnableResult:
    """
    Haptics.SetEnable 响应信息。
    """
    status_code: int
    handset_index: int
    enable: int


@dataclass
class HapticsModeResult:
    """
    Haptics.SetMode 响应信息。
    """
    status_code: int
    handset_index: int
    mode: int


@dataclass
class HapticsPressureResult:
    """
    Haptics.SetPressure 响应信息。
    """
    status_code: int
    handset_index: int
    pressure: int


@dataclass
class HapticsTimeoutResult:
    """
    Haptics.SetTimeout 响应信息。
    """
    status_code: int
    handset_index: int
    timeout_ms: int


@dataclass
class HapticsIntensityResult:
    """
    Haptics.GetIntensity 响应信息。
    """
    status_code: int
    handset_index: int
    intensity: int


# -----------------------------
# 解析工具
# -----------------------------


def _parse_telemetry_payload(payload: bytes, protocol: str) -> TelemetrySnapshot:
    """
    解析遥测载荷为 TelemetrySnapshot。
    :param payload: 遥测载荷字节
    :param protocol: 协议类型（\"legacy\"）
    :return: 解析后的遥测快照
    """
    length = len(payload)
    if length not in LEGACY_PAYLOAD_LENGTHS:
        raise ValueError(f"不支持的载荷长度: {length}")

    snap = TelemetrySnapshot(protocol=protocol, timestamp=time.time())

    offset = 0
    snap.joystick_left = list(struct.unpack_from('<4h', payload, offset))
    offset += 8
    snap.joystick_right = list(struct.unpack_from('<4h', payload, offset))
    offset += 8
    snap.arm_joint_left = list(struct.unpack_from('<8h', payload, offset))
    offset += 16
    snap.arm_joint_right = list(struct.unpack_from('<8h', payload, offset))
    offset += 16

    snap.arm_joint_left_rad = [v * ENCODER_TO_RADIAN_RATIO for v in snap.arm_joint_left]
    snap.arm_joint_right_rad = [v * ENCODER_TO_RADIAN_RATIO for v in snap.arm_joint_right]

    if length == LEGACY_PAYLOAD_LEN_BASE:
        snap.format_version = 1
        return snap

    # 躯干 IMU（加速度、角速度、四元数）
    snap.format_version = 2
    snap.torso_acc = list(struct.unpack_from('<3f', payload, offset))
    offset += 12
    snap.torso_gyro = list(struct.unpack_from('<3f', payload, offset))
    offset += 12
    snap.torso_quat = list(struct.unpack_from('<4f', payload, offset))
    offset += 16

    if length == LEGACY_PAYLOAD_LEN_TORSO_IMU_QUAT:
        return snap

    # 额外 IMU
    snap.format_version = 3
    snap.extra_acc = list(struct.unpack_from('<3f', payload, offset))
    offset += 12
    snap.extra_gyro = list(struct.unpack_from('<3f', payload, offset))
    offset += 12
    snap.extra_quat = list(struct.unpack_from('<4f', payload, offset))
    return snap


# -----------------------------
# 字节流解析器（legacy + QnTP）
# -----------------------------


class QnbotStreamParser:
    """解析混合字节流，可能同时包含legacy协议帧与 QnTP 帧。"""

    def __init__(
        self,
        max_qntp_payload: int = QNTP_MAX_PAYLOAD,
        protocol_mode: str = "auto",
    ):
        """
        初始化解析器。
        :param max_qntp_payload: 允许的 QnTP 最大载荷长度
        :param protocol_mode: 解析模式("auto"|"legacy"|"qntp")
        """
        self.buffer = bytearray()
        self.max_qntp_payload = max_qntp_payload
        self.protocol_mode = protocol_mode
        if self.protocol_mode == "legacy":
            # legacy 模式下仍允许解析 QnTP 响应帧，避免请求超时
            self.protocol_mode = "auto"
        if self.protocol_mode not in ("auto", "qntp"):
            raise ValueError("protocol_mode 仅支持: auto | legacy | qntp")
        self.legacy_frame_lengths = sorted(LEGACY_FRAME_LENGTHS, reverse=True)
        self._legacy_preferred_len: Optional[int] = None
        self._legacy_preferred_failures = 0
        self._legacy_switch_threshold = 3
        self._stats = {
            "bytes_in": 0,
            "dropped_bytes": 0,
            "qntp_ok": 0,
            "legacy_ok": 0,
            "qntp_bad_version": 0,
            "qntp_len_invalid": 0,
            "qntp_crc_fail": 0,
            "legacy_checksum_fail": 0,
            "legacy_bad_tail": 0,
            "legacy_len_switch": 0,
            "resyncs": 0,
            "unknown_sof": 0,
        }

    def feed(self, data: bytes) -> List[Union[TelemetrySnapshot, QnTPFrame]]:
        """
        输入字节流并解析为帧。
        :param data: 原始字节流
        :return: 已解析的帧列表（遥测或 QnTP 帧）
        """
        if data:
            self._stats["bytes_in"] += len(data)
            self.buffer.extend(data)
        frames: List[Union[TelemetrySnapshot, QnTPFrame]] = []
        while True:
            item = self._try_parse_one()
            if item is None:
                break
            frames.append(item)
        return frames

    def get_stats(self) -> Dict[str, int]:
        """
        获取解析统计信息。
        :return: 统计字典
        """
        return dict(self._stats)

    def _try_parse_one(self) -> Optional[Union[TelemetrySnapshot, QnTPFrame]]:
        """
        尝试从缓冲区解析一个帧。
        :return: 解析出的帧，或 None（数据不足）
        """
        def _try_qntp_at(index: int) -> Tuple[str, Optional[QnTPFrame], int]:
            """返回 (状态, 帧, 总长度)。状态: valid/invalid/need_more"""
            if len(self.buffer) < index + QNTP_HEADER_SIZE:
                return "need_more", None, 0
            version = self.buffer[index + 1]
            if version != QNTP_VERSION:
                self._stats["qntp_bad_version"] += 1
                return "invalid", None, 0
            payload_len = struct.unpack_from('<H', self.buffer, index + 6)[0]
            if payload_len > self.max_qntp_payload:
                self._stats["qntp_len_invalid"] += 1
                return "invalid", None, 0
            total_len = QNTP_HEADER_SIZE + payload_len + QNTP_CRC_SIZE
            if len(self.buffer) < index + total_len:
                return "need_more", None, 0
            frame_bytes = bytes(self.buffer[index:index + total_len])
            crc_expected = struct.unpack_from('<H', frame_bytes, total_len - 2)[0]
            crc_calc = crc16_ccitt(frame_bytes[1:total_len - 2])
            if crc_calc != crc_expected:
                self._stats["qntp_crc_fail"] += 1
                return "invalid", None, 0
            msg_type = frame_bytes[2]
            msg_class = frame_bytes[3]
            msg_id = frame_bytes[4]
            seq = frame_bytes[5]
            payload = frame_bytes[8:8 + payload_len]
            frame = QnTPFrame(
                version=version,
                msg_type=msg_type,
                msg_class=msg_class,
                msg_id=msg_id,
                seq=seq,
                payload=payload,
                raw=frame_bytes,
            )
            return "valid", frame, total_len

        def _try_legacy_at(index: int) -> Tuple[str, Optional[TelemetrySnapshot], int]:
            """返回 (状态, 帧, 总长度)。状态: valid/invalid/need_more"""
            min_len = min(self.legacy_frame_lengths)
            if len(self.buffer) < index + min_len:
                return "need_more", None, 0
            def _try_legacy_len(frame_len: int) -> Tuple[str, Optional[TelemetrySnapshot]]:
                if len(self.buffer) < index + frame_len:
                    return "need_more", None
                candidate = self.buffer[index:index + frame_len]
                if candidate[-1] != LEGACY_EOF:
                    self._stats["legacy_bad_tail"] += 1
                    return "invalid", None
                checksum = 0
                for b in candidate[1:-2]:
                    checksum ^= b
                if checksum != candidate[-2]:
                    self._stats["legacy_checksum_fail"] += 1
                    return "invalid", None
                payload = bytes(candidate[1:-2])
                try:
                    snap = _parse_telemetry_payload(payload, protocol="legacy")
                except Exception:
                    return "invalid", None
                return "valid", snap

            preferred_len = self._legacy_preferred_len
            if preferred_len is not None:
                pref_frame_len = preferred_len + 3
                status, snap = _try_legacy_len(pref_frame_len)
                if status == "need_more":
                    return "need_more", None, 0
                if status == "valid":
                    self._legacy_preferred_failures = 0
                    return "valid", snap, pref_frame_len
                self._legacy_preferred_failures += 1
                if self._legacy_preferred_failures < self._legacy_switch_threshold:
                    return "invalid", None, 0

            for frame_len in self.legacy_frame_lengths:
                if preferred_len is not None and frame_len == preferred_len + 3:
                    continue
                status, snap = _try_legacy_len(frame_len)
                if status == "need_more":
                    continue
                if status == "valid":
                    if preferred_len is None or preferred_len + 3 != frame_len:
                        self._stats["legacy_len_switch"] += 1
                    self._legacy_preferred_len = frame_len - 3
                    self._legacy_preferred_failures = 0
                    return "valid", snap, frame_len

            return "invalid", None, 0

        def _drop_to_next_sof(sof_bytes: bytes) -> None:
            idx = self.buffer.find(sof_bytes, 1)
            if idx == -1:
                self._stats["dropped_bytes"] += len(self.buffer)
                self.buffer.clear()
                return
            self._stats["dropped_bytes"] += idx
            del self.buffer[:idx]

        def _drop_to_next_any_sof() -> None:
            idx_a5 = self.buffer.find(bytes([QNTP_SOF]), 1)
            idx_aa = self.buffer.find(bytes([LEGACY_SOF]), 1)
            candidates = [i for i in (idx_a5, idx_aa) if i != -1]
            if not candidates:
                self._stats["dropped_bytes"] += len(self.buffer)
                self.buffer.clear()
                return
            idx = min(candidates)
            self._stats["dropped_bytes"] += idx
            del self.buffer[:idx]

        while True:
            if not self.buffer:
                return None

            first = self.buffer[0]

            if self.protocol_mode == "qntp":
                if first != QNTP_SOF:
                    self._stats["unknown_sof"] += 1
                    _drop_to_next_sof(bytes([QNTP_SOF]))
                    continue
                status, frame, total_len = _try_qntp_at(0)
                if status == "need_more":
                    return None
                if status == "valid":
                    del self.buffer[:total_len]
                    self._stats["qntp_ok"] += 1
                    return frame
                self._stats["resyncs"] += 1
                _drop_to_next_sof(bytes([QNTP_SOF]))
                continue

            if self.protocol_mode == "legacy":
                if first != LEGACY_SOF:
                    self._stats["unknown_sof"] += 1
                    _drop_to_next_sof(bytes([LEGACY_SOF]))
                    continue
                status, frame, total_len = _try_legacy_at(0)
                if status == "need_more":
                    return None
                if status == "valid":
                    del self.buffer[:total_len]
                    self._stats["legacy_ok"] += 1
                    return frame
                self._stats["resyncs"] += 1
                _drop_to_next_sof(bytes([LEGACY_SOF]))
                continue

            # auto
            if first == QNTP_SOF:
                status, frame, total_len = _try_qntp_at(0)
                if status == "need_more":
                    return None
                if status == "valid":
                    del self.buffer[:total_len]
                    self._stats["qntp_ok"] += 1
                    return frame
                self._stats["resyncs"] += 1
                _drop_to_next_any_sof()
                continue
            if first == LEGACY_SOF:
                status, frame, total_len = _try_legacy_at(0)
                if status == "need_more":
                    return None
                if status == "valid":
                    del self.buffer[:total_len]
                    self._stats["legacy_ok"] += 1
                    return frame
                self._stats["resyncs"] += 1
                _drop_to_next_any_sof()
                continue

            self._stats["unknown_sof"] += 1
            _drop_to_next_any_sof()
            continue


# -----------------------------
# QnTP 组帧与解码
# -----------------------------


def build_qntp_frame(
    msg_type: int,
    msg_class: int,
    msg_id: int,
    seq: int,
    payload: bytes = b"",
    version: int = QNTP_VERSION,
) -> bytes:
    """
    组装 QnTP 帧。
    :param msg_type: 消息类型（Request/Response/Notify）
    :param msg_class: 功能域
    :param msg_id: 消息 ID
    :param seq: 序号
    :param payload: 负载
    :param version: 协议版本
    :return: 完整帧字节
    """
    length = len(payload)
    header = bytes([
        QNTP_SOF,
        version,
        msg_type,
        msg_class,
        msg_id,
        seq,
    ]) + struct.pack('<H', length)
    crc = crc16_ccitt(header[1:] + payload)
    return header + payload + struct.pack('<H', crc)


def _decode_c_string(raw: bytes) -> str:
    """
    解析以 NUL 结尾的定长 ASCII 字符串字段。
    :param raw: 原始字节
    :return: 解码后的字符串
    """
    zero_pos = raw.find(b"\x00")
    if zero_pos >= 0:
        raw = raw[:zero_pos]
    return raw.decode("ascii", errors="ignore")


def _encode_fixed_ascii_string(value: Union[str, bytes], field_len: int) -> bytes:
    """
    将字符串编码为定长 ASCII 字段，超长截断，不足补零。
    :param value: 字符串或字节串
    :param field_len: 字段长度
    :return: 定长字节串
    """
    if isinstance(value, bytes):
        raw = value
    else:
        raw = value.encode("ascii", errors="ignore")
    return raw[:field_len].ljust(field_len, b"\x00")


def decode_system_version(payload: bytes) -> SystemVersionInfo:
    """
    解析 System.GetVersion 响应。
    :param payload: 响应载荷
    :return: 系统版本信息
    """
    if len(payload) < 9:
        raise ValueError("System.GetVersion 响应长度不足")
    status, proto_major, proto_minor, device_type, platform, revision, feature, build = struct.unpack_from(
        '<BBBHBBBB', payload, 0
    )
    battery_mv = 0
    battery_percent = 0
    wireless_link_state = 0
    serial = b""
    device_name = ""

    if len(payload) >= 12:
        battery_mv = struct.unpack_from('<H', payload, 9)[0]
        battery_percent = payload[11]

    if len(payload) >= 13:
        wireless_link_state = payload[12]

    if len(payload) >= 25:
        serial = payload[13:25]
        if len(payload) > 25:
            device_name = _decode_c_string(payload[25:])

    return SystemVersionInfo(
        status_code=status,
        proto_major=proto_major,
        proto_minor=proto_minor,
        device_type=device_type,
        platform=platform,
        revision=revision,
        feature=feature,
        build=build,
        battery_mv=battery_mv,
        battery_percent=battery_percent,
        wireless_link_state=wireless_link_state,
        serial=serial,
        device_name=device_name,
    )


def decode_system_caps(payload: bytes) -> SystemCapabilities:
    """
    解析 System.GetCapabilities 响应。
    :param payload: 响应载荷
    :return: 系统能力信息
    """
    if len(payload) < 41:
        raise ValueError("System.GetCapabilities 响应长度不足")
    status = payload[0]
    supported = payload[1:33]
    return SystemCapabilities(status_code=status, supported_class=supported)


def decode_system_status(payload: bytes) -> SystemStatusInfo:
    """
    解析 System.Status 响应。
    :param payload: 响应载荷
    :return: 系统状态信息
    """
    fixed_len = 49
    if len(payload) < fixed_len:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return SystemStatusInfo(
                status_code=payload[0],
                battery_mv=0,
                battery_percent=0,
                wireless_link_state=0,
                serial=b"",
                device_name="",
            )
        raise ValueError("System.Status 响应长度不足")

    status = payload[0]
    battery_mv = struct.unpack_from('<H', payload, 1)[0]
    battery_percent = payload[3]
    wireless_link_state = payload[4]
    serial = payload[5:17]
    device_name = _decode_c_string(payload[17:49])
    return SystemStatusInfo(
        status_code=status,
        battery_mv=battery_mv,
        battery_percent=battery_percent,
        wireless_link_state=wireless_link_state,
        serial=serial,
        device_name=device_name,
    )


def decode_device_profile_topology(payload: bytes) -> DeviceProfileTopology:
    """
    解析 DeviceProfile.GetTopology 响应。
    :param payload: 响应载荷
    :return: 设备拓扑信息
    """
    # 当前协议字段最小长度为 7B:
    # status + enc_count + enc_slots + hand_count + hand_slots + imu_count + imu_slots
    if len(payload) < 7:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return DeviceProfileTopology(
                status_code=payload[0],
                encoder_count=0,
                encoder_total_slots=0,
                handset_count=0,
                handset_total_slots=0,
                imu_count=0,
                imu_total_slots=0,
            )
        raise ValueError("DeviceProfile.GetTopology 响应长度不足")
    return DeviceProfileTopology(
        status_code=payload[0],
        encoder_count=payload[1],
        encoder_total_slots=payload[2],
        handset_count=payload[3],
        handset_total_slots=payload[4],
        imu_count=payload[5],
        imu_total_slots=payload[6],
    )


def decode_device_profile_data_profiles(payload: bytes) -> DeviceProfileDataProfiles:
    """
    解析 DeviceProfile.GetDataProfiles 响应。
    当前固件通常返回 UNSUPPORTED，保留原始 payload 以便上层自行处理。
    :param payload: 响应载荷
    :return: 数据 profile 响应
    """
    if len(payload) < 1:
        raise ValueError("DeviceProfile.GetDataProfiles 响应长度不足")
    return DeviceProfileDataProfiles(status_code=payload[0], raw_payload=payload[1:])


def decode_telemetry_stream_runtime_config(payload: bytes) -> TelemetryStreamRuntimeConfigResult:
    """
    解析 Telemetry.SetStreamRuntimeConfig 响应。
    :param payload: 响应载荷
    :return: 流配置结果
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return TelemetryStreamRuntimeConfigResult(
                status_code=payload[0],
                stream_id=payload[1] if len(payload) >= 2 else 0,
            )
        raise ValueError("Telemetry.SetStreamRuntimeConfig 响应长度不足")
    return TelemetryStreamRuntimeConfigResult(
        status_code=payload[0],
        stream_id=payload[1],
    )


def decode_encoder_info(payload: bytes) -> EncoderInfo:
    """
    解析 Encoder.GetInfo 响应。
    :param payload: 响应载荷
    :return: 编码器信息
    """
    if len(payload) < 20:
        if len(payload) >= 1 and payload[0] != 0x00:
            return EncoderInfo(
                status_code=payload[0],
                channel_index=0,
                encoder_id=0,
                hw_version=0,
                fw_version=0,
                serial=b"",
            )
        raise ValueError("Encoder.GetInfo 响应长度不足")
    status = payload[0]
    channel_index = payload[1]
    encoder_id = payload[2]
    hw_version = struct.unpack_from('<H', payload, 4)[0]
    fw_version = struct.unpack_from('<H', payload, 6)[0]
    serial = payload[8:20]
    return EncoderInfo(
        status_code=status,
        channel_index=channel_index,
        encoder_id=encoder_id,
        hw_version=hw_version,
        fw_version=fw_version,
        serial=serial,
    )


def decode_encoder_info_list(payload: bytes) -> EncoderInfoList:
    """
    解析 Encoder.GetInfoList 响应。
    :param payload: 响应载荷
    :return: 编码器列表
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != 0x00:
            return EncoderInfoList(status_code=payload[0], entries=[])
        raise ValueError("Encoder.GetInfoList 响应长度不足")
    status = payload[0]
    count = payload[1]
    entries: List[EncoderInfoEntry] = []
    offset = 4
    for _ in range(count):
        if len(payload) < offset + 18:
            break
        channel_index = payload[offset]
        encoder_id = payload[offset + 1]
        hw_version = struct.unpack_from('<H', payload, offset + 2)[0]
        fw_version = struct.unpack_from('<H', payload, offset + 4)[0]
        serial = payload[offset + 6: offset + 18]
        entries.append(EncoderInfoEntry(
            channel_index=channel_index,
            encoder_id=encoder_id,
            hw_version=hw_version,
            fw_version=fw_version,
            serial=serial,
        ))
        offset += 18
    return EncoderInfoList(status_code=status, entries=entries)


def decode_encoder_zero_value(payload: bytes) -> EncoderZeroValue:
    """
    解析 Encoder.GetZeroValue 响应。
    :param payload: 响应载荷
    :return: 编码器零位信息
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != 0x00:
            return EncoderZeroValue(status_code=payload[0], channel_index=0, zero_value=0)
        raise ValueError("Encoder.GetZeroValue 响应长度不足")
    status = payload[0]
    channel_index = payload[1]
    zero_value = struct.unpack_from('<H', payload, 2)[0]
    return EncoderZeroValue(status_code=status, channel_index=channel_index, zero_value=zero_value)


def decode_encoder_set_zero_here(payload: bytes) -> EncoderSetZeroHereResult:
    """
    解析 Encoder.SetZeroHere 响应。
    :param payload: 响应载荷
    :return: 设零结果
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != 0x00:
            return EncoderSetZeroHereResult(
                status_code=payload[0],
                channel_index=0,
                zero_offset=0,
                angle_raw=None,
            )
        raise ValueError("Encoder.SetZeroHere 响应长度不足")
    status = payload[0]
    channel_index = payload[1]
    zero_offset = struct.unpack_from('<H', payload, 2)[0]
    # 兼容两种固件返回：
    # - 当前主线：4 字节（status, ch, zero_offset）
    # - 历史版本：6 字节（额外带 angle_raw）
    angle_raw = struct.unpack_from('<H', payload, 4)[0] if len(payload) >= 6 else None
    return EncoderSetZeroHereResult(
        status_code=status,
        channel_index=channel_index,
        zero_offset=zero_offset,
        angle_raw=angle_raw,
    )


def decode_encoder_set_zero_value(payload: bytes) -> EncoderSetZeroValueResult:
    """
    解析 Encoder.SetZeroValue 响应。
    :param payload: 响应载荷
    :return: 设零值结果
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != 0x00:
            return EncoderSetZeroValueResult(status_code=payload[0], channel_index=0)
        raise ValueError("Encoder.SetZeroValue 响应长度不足")
    status = payload[0]
    channel_index = payload[1]
    return EncoderSetZeroValueResult(status_code=status, channel_index=channel_index)


def decode_hand_info(payload: bytes) -> HandInfo:
    """
    解析 Hand.GetInfo 响应。
    :param payload: 响应载荷
    :return: 手柄信息
    """
    if len(payload) < 18:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HandInfo(status_code=payload[0], handset_index=0, hw_version=0, fw_version=0, serial=b"")
        raise ValueError("Hand.GetInfo 响应长度不足")
    return HandInfo(
        status_code=payload[0],
        handset_index=payload[1],
        hw_version=struct.unpack_from('<H', payload, 2)[0],
        fw_version=struct.unpack_from('<H', payload, 4)[0],
        serial=payload[6:18],
    )


def decode_hand_info_list(payload: bytes) -> HandInfoList:
    """
    解析 Hand.GetInfoList 响应。
    :param payload: 响应载荷
    :return: 手柄列表
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HandInfoList(status_code=payload[0], entries=[])
        raise ValueError("Hand.GetInfoList 响应长度不足")
    status = payload[0]
    count = payload[1]
    entries: List[HandInfoEntry] = []
    offset = 4
    for _ in range(count):
        if len(payload) < offset + 17:
            break
        entries.append(
            HandInfoEntry(
                handset_index=payload[offset],
                hw_version=struct.unpack_from('<H', payload, offset + 1)[0],
                fw_version=struct.unpack_from('<H', payload, offset + 3)[0],
                serial=payload[offset + 5:offset + 17],
            )
        )
        offset += 17
    return HandInfoList(status_code=status, entries=entries)


def decode_hand_calib_params(payload: bytes) -> HandCalibParams:
    """
    解析 Hand.GetCalibParams 响应。
    :param payload: 响应载荷
    :return: 手柄校准参数
    """
    if len(payload) < 20:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HandCalibParams(
                status_code=payload[0],
                handset_index=0,
                calib_output_enabled=0,
                jx_center=0,
                jx_max=0,
                jx_min=0,
                jy_center=0,
                jy_max=0,
                jy_min=0,
                trig_start=0,
                trig_max=0,
            )
        raise ValueError("Hand.GetCalibParams 响应长度不足")
    return HandCalibParams(
        status_code=payload[0],
        handset_index=payload[1],
        calib_output_enabled=payload[2],
        jx_center=struct.unpack_from('<H', payload, 4)[0],
        jx_max=struct.unpack_from('<H', payload, 6)[0],
        jx_min=struct.unpack_from('<H', payload, 8)[0],
        jy_center=struct.unpack_from('<H', payload, 10)[0],
        jy_max=struct.unpack_from('<H', payload, 12)[0],
        jy_min=struct.unpack_from('<H', payload, 14)[0],
        trig_start=struct.unpack_from('<H', payload, 16)[0],
        trig_max=struct.unpack_from('<H', payload, 18)[0],
    )


def decode_hand_command_result(payload: bytes, command_name: str = "Hand") -> HandCommandResult:
    """
    解析 Hand 动作类通用响应。
    :param payload: 响应载荷
    :param command_name: 命令名称
    :return: 通用响应
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HandCommandResult(status_code=payload[0], handset_index=payload[1] if len(payload) >= 2 else 0)
        raise ValueError(f"{command_name} 响应长度不足")
    return HandCommandResult(status_code=payload[0], handset_index=payload[1])


def decode_hand_calib_commit_result(payload: bytes) -> HandCalibCommitResult:
    """
    解析 Hand.CalibCommit 响应。
    :param payload: 响应载荷
    :return: 提交结果
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HandCalibCommitResult(status_code=payload[0], handset_index=0, calib_point=0)
        raise ValueError("Hand.CalibCommit 响应长度不足")
    return HandCalibCommitResult(
        status_code=payload[0],
        handset_index=payload[1],
        calib_point=payload[2],
    )


def decode_hand_set_calib_output(payload: bytes) -> HandSetCalibOutputResult:
    """
    解析 Hand.SetCalibOutput 响应。
    :param payload: 响应载荷
    :return: 输出开关设置结果
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HandSetCalibOutputResult(status_code=payload[0], handset_index=0, enable=0)
        raise ValueError("Hand.SetCalibOutput 响应长度不足")
    return HandSetCalibOutputResult(
        status_code=payload[0],
        handset_index=payload[1],
        enable=payload[2],
    )


def decode_hand_set_calib_param(payload: bytes) -> HandSetCalibParamResult:
    """
    解析 Hand.SetCalibParam 响应。
    :param payload: 响应载荷
    :return: 参数设置结果
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HandSetCalibParamResult(status_code=payload[0], handset_index=0, calib_point=0)
        raise ValueError("Hand.SetCalibParam 响应长度不足")
    return HandSetCalibParamResult(
        status_code=payload[0],
        handset_index=payload[1],
        calib_point=payload[2],
    )


def decode_imu_info(payload: bytes) -> ImuInfo:
    """
    解析 Imu.GetInfo 响应。
    :param payload: 响应载荷
    :return: IMU 信息
    """
    if len(payload) < 6:
        if len(payload) >= 1 and payload[0] != 0x00:
            return ImuInfo(status_code=payload[0], imu_index=0, version=0)
        raise ValueError("Imu.GetInfo 响应长度不足")
    status = payload[0]
    imu_index = payload[1]
    version = struct.unpack_from('<I', payload, 2)[0]
    return ImuInfo(status_code=status, imu_index=imu_index, version=version)


def decode_imu_info_list(payload: bytes) -> ImuInfoList:
    """
    解析 Imu.GetInfoList 响应。
    :param payload: 响应载荷
    :return: IMU 列表
    """
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != 0x00:
            return ImuInfoList(status_code=payload[0], entries=[])
        raise ValueError("Imu.GetInfoList 响应长度不足")
    status = payload[0]
    count = payload[1]
    entries: List[ImuInfoEntry] = []
    offset = 4
    for _ in range(count):
        if len(payload) < offset + 5:
            break
        imu_index = payload[offset]
        version = struct.unpack_from('<I', payload, offset + 1)[0]
        entries.append(ImuInfoEntry(imu_index=imu_index, version=version))
        offset += 5
    return ImuInfoList(status_code=status, entries=entries)


def decode_imu_mag_calibrate(payload: bytes) -> ImuMagCalibrateResult:
    """
    解析 Imu.MagCalibrate 响应。
    :param payload: 响应载荷
    :return: 磁力计校准结果
    """
    if len(payload) < 6:
        if len(payload) >= 1 and payload[0] != 0x00:
            return ImuMagCalibrateResult(
                status_code=payload[0],
                action_echo=0,
                imu_index=0,
                state=0,
                error_code=0,
            )
        raise ValueError("Imu.MagCalibrate 响应长度不足")
    status = payload[0]
    action_echo = payload[1]
    imu_index = payload[2]
    state = payload[3]
    error_code = struct.unpack_from('<H', payload, 4)[0]
    return ImuMagCalibrateResult(
        status_code=status,
        action_echo=action_echo,
        imu_index=imu_index,
        state=state,
        error_code=error_code,
    )


def decode_imu_output_enable(payload: bytes) -> ImuOutputEnableResult:
    """
    解析 Imu.SetOutputEnable / Imu.GetOutputEnable 响应。
    :param payload: 响应载荷
    :return: IMU 输出开关结果
    """
    if len(payload) < 6:
        if len(payload) >= 1 and payload[0] != 0x00:
            return ImuOutputEnableResult(
                status_code=payload[0],
                enable=0,
                persisted=0,
                capability_mask=0,
                error_code=0,
            )
        raise ValueError("Imu.OutputEnable 响应长度不足")
    status, enable, persisted, capability_mask, error_code, reserved = struct.unpack_from('<6B', payload, 0)
    return ImuOutputEnableResult(
        status_code=status,
        enable=enable,
        persisted=persisted,
        capability_mask=capability_mask,
        error_code=error_code,
        reserved=reserved,
    )


def decode_wireless_status_info(payload: bytes) -> WirelessStatusInfo:
    """
    解析 Wireless.GetStatusInfo 响应。
    :param payload: 响应载荷
    :return: Wireless 状态信息
    """
    fixed_len = 12
    if len(payload) < fixed_len:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return WirelessStatusInfo(
                status_code=payload[0],
                pairing_busy=0,
                pair_step=0,
                pair_result=0,
                in_config_mode=0,
                link_state=0,
                passthrough_enabled=0,
                push_freq_option=0,
            )
        raise ValueError("Wireless.GetStatusInfo 响应长度不足")

    (
        status,
        pairing_busy,
        pair_step,
        pair_result,
        in_config_mode,
        link_state,
        passthrough_enabled,
        push_freq_option,
    ) = struct.unpack_from('<8B', payload, 0)

    remaining = payload[fixed_len:]
    if remaining and (len(remaining) % 2 != 0):
        raise ValueError("Wireless.GetStatusInfo 名称字段长度非法")

    name_field_len = len(remaining) // 2
    local_name = ""
    paired_name = ""
    if name_field_len > 0:
        local_name = _decode_c_string(remaining[:name_field_len])
        paired_name = _decode_c_string(remaining[name_field_len:name_field_len * 2])

    return WirelessStatusInfo(
        status_code=status,
        pairing_busy=pairing_busy,
        pair_step=pair_step,
        pair_result=pair_result,
        in_config_mode=in_config_mode,
        link_state=link_state,
        passthrough_enabled=passthrough_enabled,
        push_freq_option=push_freq_option,
        local_device_name=local_name,
        paired_device_name=paired_name,
    )


def decode_wireless_command_result(payload: bytes, command_name: str = "Wireless") -> WirelessCommandResult:
    """
    解析 Wireless 动作/设置类命令响应。
    :param payload: 响应载荷
    :param command_name: 命令名称，仅用于报错提示
    :return: 通用状态响应
    """
    if len(payload) < 1:
        raise ValueError(f"{command_name} 响应长度不足")
    return WirelessCommandResult(status_code=payload[0])


def decode_haptics_output_result(payload: bytes) -> HapticsOutputResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsOutputResult(status_code=payload[0], channel_id=payload[1] if len(payload) >= 2 else 0)
        raise ValueError("Haptics.SetOutput 响应长度不足")
    return HapticsOutputResult(status_code=payload[0], channel_id=payload[1])


def decode_haptics_drv_cal_status(payload: bytes) -> HapticsDrvCalStatus:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsDrvCalStatus(status_code=payload[0], handset_index=0, calibrated=0)
        raise ValueError("Haptics.DrvGetCalStatus 响应长度不足")
    return HapticsDrvCalStatus(status_code=payload[0], handset_index=payload[1], calibrated=payload[2])


def decode_haptics_drv_calibrate(payload: bytes) -> HapticsDrvCalibrateResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsDrvCalibrateResult(status_code=payload[0], handset_index=0, result=0)
        raise ValueError("Haptics.DrvCalibrate 响应长度不足")
    return HapticsDrvCalibrateResult(status_code=payload[0], handset_index=payload[1], result=payload[2])


def decode_haptics_vibrate_play(payload: bytes) -> HapticsVibratePlayResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsVibratePlayResult(status_code=payload[0], handset_index=0, effect_id=0)
        raise ValueError("Haptics.VibratePlay 响应长度不足")
    return HapticsVibratePlayResult(status_code=payload[0], handset_index=payload[1], effect_id=payload[2])


def decode_haptics_vibrate_stop(payload: bytes) -> HapticsVibrateStopResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsVibrateStopResult(status_code=payload[0], handset_index=0)
        raise ValueError("Haptics.VibrateStop 响应长度不足")
    return HapticsVibrateStopResult(status_code=payload[0], handset_index=payload[1])


def decode_haptics_vibrate_realtime(payload: bytes) -> HapticsVibrateRealtimeResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsVibrateRealtimeResult(status_code=payload[0], handset_index=0, amplitude=0)
        raise ValueError("Haptics.VibrateRealtime 响应长度不足")
    return HapticsVibrateRealtimeResult(status_code=payload[0], handset_index=payload[1], amplitude=payload[2])


def decode_haptics_enable(payload: bytes) -> HapticsEnableResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsEnableResult(status_code=payload[0], handset_index=0, enable=0)
        raise ValueError("Haptics.SetEnable 响应长度不足")
    return HapticsEnableResult(status_code=payload[0], handset_index=payload[1], enable=payload[2])


def decode_haptics_mode(payload: bytes) -> HapticsModeResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsModeResult(status_code=payload[0], handset_index=0, mode=0)
        raise ValueError("Haptics.SetMode 响应长度不足")
    return HapticsModeResult(status_code=payload[0], handset_index=payload[1], mode=payload[2])


def decode_haptics_pressure(payload: bytes) -> HapticsPressureResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsPressureResult(status_code=payload[0], handset_index=0, pressure=0)
        raise ValueError("Haptics.SetPressure 响应长度不足")
    return HapticsPressureResult(
        status_code=payload[0],
        handset_index=payload[1],
        pressure=struct.unpack_from('<H', payload, 2)[0],
    )


def decode_haptics_timeout(payload: bytes) -> HapticsTimeoutResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsTimeoutResult(status_code=payload[0], handset_index=0, timeout_ms=0)
        raise ValueError("Haptics.SetTimeout 响应长度不足")
    return HapticsTimeoutResult(
        status_code=payload[0],
        handset_index=payload[1],
        timeout_ms=struct.unpack_from('<H', payload, 2)[0],
    )


def decode_haptics_intensity(payload: bytes) -> HapticsIntensityResult:
    if len(payload) < 4:
        if len(payload) >= 1 and payload[0] != STATUS_OK:
            return HapticsIntensityResult(status_code=payload[0], handset_index=0, intensity=0)
        raise ValueError("Haptics.GetIntensity 响应长度不足")
    return HapticsIntensityResult(status_code=payload[0], handset_index=payload[1], intensity=payload[2])




# -----------------------------
# 串口客户端 API
# -----------------------------


class QnTPTimeoutError(RuntimeError):
    """QnTP 请求超时异常。"""
    pass


class _PendingRequest:
    def __init__(self, msg_class: int, msg_id: int):
        """
        待响应请求占位。
        :param msg_class: 功能域
        :param msg_id: 消息 ID
        """
        self.event = threading.Event()
        self.response: Optional[QnTPFrame] = None
        self.msg_class = msg_class
        self.msg_id = msg_id


class QnbotClient:
    """Qnbot 串口高层 SDK。"""

    def __init__(
        self,
        port: str,
        baudrate: int = 2_000_000,
        read_chunk_size: int = 256,
        read_timeout: float = 0.02,
        protocol_mode: str = "auto",
        max_qntp_payload: int = QNTP_MAX_PAYLOAD,
        dispatch_queue_size: int = 2048,
        on_error: Optional[Callable[[Exception], None]] = None,
    ):
        """
        初始化客户端。
        :param port: 串口路径
        :param baudrate: 波特率
        :param read_chunk_size: 单次读取字节数
        :param read_timeout: 读取超时（秒）
        :param protocol_mode: 解析模式("auto"|"legacy"|"qntp")
        :param max_qntp_payload: 允许的 QnTP 最大载荷长度
        :param dispatch_queue_size: 回调分发队列容量（防止回调阻塞接收线程）
        :param on_error: 串口打开失败回调
        """
        if serial is None:
            raise RuntimeError("需要 pyserial。请执行 `pip install pyserial` 安装。")
        self.port = port
        self.baudrate = baudrate
        self.read_chunk_size = read_chunk_size
        self.read_timeout = read_timeout
        self._on_error = on_error

        self._serial = None
        self._parser = QnbotStreamParser(
            max_qntp_payload=max_qntp_payload,
            protocol_mode=protocol_mode,
        )
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._dispatch_thread: Optional[threading.Thread] = None
        self._dispatch_queue: "queue.Queue[Tuple[str, Union[TelemetrySnapshot, QnTPFrame]]]" = (
            queue.Queue(maxsize=dispatch_queue_size)
        )
        self._dispatch_dropped = 0

        self._write_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending: Dict[int, _PendingRequest] = {}
        self._seq = 0
        self._late_lock = threading.Lock()
        self._late_responses: List[Tuple[float, QnTPFrame]] = []
        self._bytes_read = 0

        self._latest_telemetry: Optional[TelemetrySnapshot] = None
        self._telemetry_callbacks: List[Callable[[TelemetrySnapshot], None]] = []
        self._qntp_callbacks: List[Callable[[QnTPFrame], None]] = []
        self._notify_callbacks: List[Callable[[QnTPFrame], None]] = []

    # ---- 生命周期 ----

    def start(self) -> bool:
        """
        打开串口并启动接收线程。
        :return: 是否成功启动
        """
        if self._running:
            return True
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.read_timeout,
            )
        except Exception as e:
            if self._on_error:
                try:
                    self._on_error(e)
                except Exception:
                    pass
            raise
        self._running = True
        self._dispatch_thread = threading.Thread(target=self._dispatch_loop, daemon=True)
        self._dispatch_thread.start()
        self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._rx_thread.start()
        return True

    def stop(self) -> None:
        """
        停止接收线程并关闭串口。
        :return: None
        """
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._dispatch_thread:
            self._dispatch_thread.join(timeout=1.0)
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def __enter__(self) -> "QnbotClient":
        """
        上下文管理进入。
        :return: self
        """
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """
        上下文管理退出。
        :param exc_type: 异常类型
        :param exc: 异常对象
        :param tb: Traceback
        :return: None
        """
        self.stop()

    # ---- 回调 ----

    def register_telemetry_callback(self, cb: Callable[[TelemetrySnapshot], None]) -> None:
        """
        注册遥测回调。
        :param cb: 回调函数
        :return: None
        """
        self._telemetry_callbacks.append(cb)

    def unregister_telemetry_callback(self, cb: Callable[[TelemetrySnapshot], None]) -> None:
        """
        取消遥测回调。
        :param cb: 回调函数
        :return: None
        """
        if cb in self._telemetry_callbacks:
            self._telemetry_callbacks.remove(cb)

    # 旧风格别名（便于迁移）
    def register_callback(self, cb: Callable[[TelemetrySnapshot], None]) -> None:
        """
        旧版接口别名：注册遥测回调。
        :param cb: 回调函数
        :return: None
        """
        self.register_telemetry_callback(cb)

    def unregister_callback(self, cb: Callable[[TelemetrySnapshot], None]) -> None:
        """
        旧版接口别名：取消遥测回调。
        :param cb: 回调函数
        :return: None
        """
        self.unregister_telemetry_callback(cb)

    def register_qntp_callback(self, cb: Callable[[QnTPFrame], None]) -> None:
        """
        注册 QnTP 原始帧回调。
        :param cb: 回调函数
        :return: None
        """
        self._qntp_callbacks.append(cb)

    def unregister_qntp_callback(self, cb: Callable[[QnTPFrame], None]) -> None:
        """
        取消 QnTP 原始帧回调。
        :param cb: 回调函数
        :return: None
        """
        if cb in self._qntp_callbacks:
            self._qntp_callbacks.remove(cb)

    def register_notify_callback(self, cb: Callable[[QnTPFrame], None]) -> None:
        """
        注册通知回调（原始 QnTP Notify 帧）。
        :param cb: 回调函数
        :return: None
        """
        self._notify_callbacks.append(cb)

    def unregister_notify_callback(self, cb: Callable[[QnTPFrame], None]) -> None:
        """
        取消通知回调。
        :param cb: 回调函数
        :return: None
        """
        if cb in self._notify_callbacks:
            self._notify_callbacks.remove(cb)

    # ---- 数据接口 ----

    def get_latest_telemetry(self) -> Optional[TelemetrySnapshot]:
        """
        获取最新遥测数据。
        :return: 遥测快照或 None
        """
        return self._latest_telemetry

    def get_latest_data(self) -> Optional[TelemetrySnapshot]:
        """
        旧版接口别名：获取最新遥测数据。
        :return: 遥测快照或 None
        """
        return self._latest_telemetry

    def get_stats(self) -> Dict[str, Dict[str, int]]:
        """
        获取统计信息（字节流与解析统计）。
        :return: 统计字典
        """
        return {
            "bytes_read": {"total": self._bytes_read},
            "parser": self._parser.get_stats(),
            "dispatch": {
                "dropped": self._dispatch_dropped,
                "queue": self._dispatch_queue.qsize(),
            },
        }

    # ---- QnTP 高层请求 ----

    def get_system_version(self, timeout: float = 2.0) -> SystemVersionInfo:
        """
        获取系统版本信息。
        :param timeout: 超时（秒）
        :return: 系统版本信息
        """
        frame = self._request(MSGCLASS_SYSTEM, MSGID_SYSTEM_GET_VERSION, b"", timeout)
        return decode_system_version(frame.payload)

    def get_system_capabilities(self, timeout: float = 2.0) -> SystemCapabilities:
        """
        获取系统能力位图。
        :param timeout: 超时（秒）
        :return: 系统能力信息
        """
        frame = self._request(MSGCLASS_SYSTEM, MSGID_SYSTEM_GET_CAPS, b"", timeout)
        return decode_system_caps(frame.payload)

    def get_system_status(self, timeout: float = 2.0) -> SystemStatusInfo:
        """
        获取系统状态信息（电池/无线/SN/设备名）。
        :param timeout: 超时（秒）
        :return: 系统状态信息
        """
        frame = self._request(MSGCLASS_SYSTEM, MSGID_SYSTEM_STATUS, b"", timeout)
        return decode_system_status(frame.payload)

    def enter_system_dfu(self, timeout: float = 2.0) -> int:
        """
        请求设备进入系统 DFU（Bootloader）模式。
        :param timeout: 超时（秒）
        :return: status_code（0 表示 OK）
        """
        frame = self._request(MSGCLASS_SYSTEM, MSGID_SYSTEM_ENTER_DFU, b"", timeout)
        if len(frame.payload) < 1:
            raise ValueError("System.EnterDfu 响应长度不足")
        return frame.payload[0]

    def get_device_profile_topology(self, timeout: float = 2.0) -> DeviceProfileTopology:
        """
        获取设备拓扑信息。
        :param timeout: 超时（秒）
        :return: 设备拓扑
        """
        frame = self._request(MSGCLASS_DEVICE_PROFILE, MSGID_DEVICE_PROFILE_GET_TOPOLOGY, b"", timeout)
        return decode_device_profile_topology(frame.payload)

    def get_device_profile_data_profiles(self, timeout: float = 2.0) -> DeviceProfileDataProfiles:
        """
        获取数据流 profile 信息。
        :param timeout: 超时（秒）
        :return: 数据 profile 响应
        """
        frame = self._request(MSGCLASS_DEVICE_PROFILE, MSGID_DEVICE_PROFILE_GET_DATA_PROFILES, b"", timeout)
        return decode_device_profile_data_profiles(frame.payload)

    def set_telemetry_stream_runtime_config(
        self,
        stream_id: int,
        enable: Union[int, bool],
        timeout: float = 2.0,
    ) -> TelemetryStreamRuntimeConfigResult:
        """
        设置 Telemetry 运行时流配置。
        :param stream_id: 流 ID，0xF1=LegacyProtocol，0x00=HighRateSnapshot
        :param enable: 是否启用
        :param timeout: 超时（秒）
        :return: 设置结果
        """
        sid = int(stream_id) & 0xFF
        if sid not in (TELEMETRY_STREAM_ID_LEGACY_PROTOCOL, TELEMETRY_STREAM_ID_HIGH_RATE_SNAPSHOT):
            raise ValueError("stream_id 仅支持 0xF1(LegacyProtocol) 或 0x00(HighRateSnapshot)")
        payload = bytes([sid, 1 if bool(enable) else 0, 0, 0])
        frame = self._request(MSGCLASS_TELEMETRY, MSGID_TELEMETRY_SET_STREAM_RUNTIME_CONFIG, payload, timeout)
        return decode_telemetry_stream_runtime_config(frame.payload)

    def get_encoder_info(self, channel_index: int, timeout: float = 2.0) -> EncoderInfo:
        """
        获取单个编码器信息。
        :param channel_index: 编码器通道号
        :param timeout: 超时（秒）
        :return: 编码器信息
        """
        payload = bytes([channel_index & 0xFF, 0])
        frame = self._request(MSGCLASS_ENCODER, MSGID_ENCODER_GET_INFO, payload, timeout)
        return decode_encoder_info(frame.payload)

    def get_encoder_info_list(self, timeout: float = 2.0) -> EncoderInfoList:
        """
        获取编码器列表。
        :param timeout: 超时（秒）
        :return: 编码器列表
        """
        payload = bytes([0, 0])
        frame = self._request(MSGCLASS_ENCODER, MSGID_ENCODER_GET_INFO_LIST, payload, timeout)
        return decode_encoder_info_list(frame.payload)

    def get_encoder_zero_value(self, channel_index: int, timeout: float = 2.0) -> EncoderZeroValue:
        """
        获取编码器零位。
        :param channel_index: 编码器通道号
        :param timeout: 超时（秒）
        :return: 零位信息
        """
        payload = bytes([channel_index & 0xFF])
        frame = self._request(MSGCLASS_ENCODER, MSGID_ENCODER_GET_ZERO, payload, timeout)
        return decode_encoder_zero_value(frame.payload)

    def set_encoder_zero_here(self, channel_index: int, timeout: float = 2.0) -> EncoderSetZeroHereResult:
        """
        当前位置设为编码器零位。
        :param channel_index: 编码器通道号
        :param timeout: 超时（秒）
        :return: 设零结果
        """
        payload = bytes([channel_index & 0xFF, 0])
        frame = self._request(MSGCLASS_ENCODER, MSGID_ENCODER_SET_ZERO_HERE, payload, timeout)
        return decode_encoder_set_zero_here(frame.payload)

    def set_encoder_zero_value(self, channel_index: int, zero_value: int, timeout: float = 2.0) -> EncoderSetZeroValueResult:
        """
        设置编码器零位值。
        :param channel_index: 编码器通道号
        :param zero_value: 零位值
        :param timeout: 超时（秒）
        :return: 设零结果
        """
        # 当前固件请求解析按高字节在前读取 zero_value（payload[2]<<8 | payload[3]）。
        payload = bytes([channel_index & 0xFF, 0]) + struct.pack('>H', zero_value & 0xFFFF)
        frame = self._request(MSGCLASS_ENCODER, MSGID_ENCODER_SET_ZERO_VALUE, payload, timeout)
        return decode_encoder_set_zero_value(frame.payload)

    def get_hand_info(self, handset_index: int, timeout: float = 2.0) -> HandInfo:
        """
        获取单个手柄信息。
        :param handset_index: 手柄索引
        :param timeout: 超时（秒）
        :return: 手柄信息
        """
        payload = bytes([handset_index & 0xFF, 0])
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_GET_INFO, payload, timeout)
        return decode_hand_info(frame.payload)

    def get_hand_info_list(self, timeout: float = 2.0) -> HandInfoList:
        """
        获取手柄列表。
        :param timeout: 超时（秒）
        :return: 手柄列表
        """
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_GET_INFO_LIST, bytes([0, 0]), timeout)
        return decode_hand_info_list(frame.payload)

    def get_hand_calib_params(self, handset_index: int, timeout: float = 2.0) -> HandCalibParams:
        """
        获取手柄校准参数。
        :param handset_index: 手柄索引
        :param timeout: 超时（秒）
        :return: 手柄校准参数
        """
        payload = bytes([handset_index & 0xFF])
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_GET_CALIB_PARAMS, payload, timeout)
        return decode_hand_calib_params(frame.payload)

    def hand_calib_start(self, handset_index: int, timeout: float = 2.0) -> HandCommandResult:
        """
        开始手柄校准。
        :param handset_index: 手柄索引
        :param timeout: 超时（秒）
        :return: 通用结果
        """
        payload = bytes([handset_index & 0xFF, 0, 0, 0])
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_CALIB_START, payload, timeout)
        return decode_hand_command_result(frame.payload, "Hand.CalibStart")

    def hand_calib_commit(self, handset_index: int, calib_point: int, timeout: float = 2.0) -> HandCalibCommitResult:
        """
        提交手柄校准点。
        :param handset_index: 手柄索引
        :param calib_point: 校准点
        :param timeout: 超时（秒）
        :return: 提交结果
        """
        payload = bytes([handset_index & 0xFF, calib_point & 0xFF, 0, 0])
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_CALIB_COMMIT, payload, timeout)
        return decode_hand_calib_commit_result(frame.payload)

    def hand_calib_finish(self, handset_index: int, timeout: float = 2.0) -> HandCommandResult:
        """
        结束手柄校准。
        :param handset_index: 手柄索引
        :param timeout: 超时（秒）
        :return: 通用结果
        """
        payload = bytes([handset_index & 0xFF, 0, 0, 0])
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_CALIB_FINISH, payload, timeout)
        return decode_hand_command_result(frame.payload, "Hand.CalibFinish")

    def hand_set_calib_output(self, handset_index: int, enable: Union[int, bool], timeout: float = 2.0) -> HandSetCalibOutputResult:
        """
        设置手柄校准输出开关。
        :param handset_index: 手柄索引
        :param enable: 开关值
        :param timeout: 超时（秒）
        :return: 设置结果
        """
        payload = bytes([handset_index & 0xFF, 1 if bool(enable) else 0, 0, 0])
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_SET_CALIB_OUTPUT, payload, timeout)
        return decode_hand_set_calib_output(frame.payload)

    def hand_set_calib_param(self, handset_index: int, calib_point: int, value: int, timeout: float = 2.0) -> HandSetCalibParamResult:
        """
        设置手柄校准点数值。
        :param handset_index: 手柄索引
        :param calib_point: 校准点
        :param value: 目标值
        :param timeout: 超时（秒）
        :return: 设置结果
        """
        payload = bytes([handset_index & 0xFF, calib_point & 0xFF]) + struct.pack('<H', value & 0xFFFF)
        frame = self._request(MSGCLASS_HANDSET, MSGID_HAND_SET_CALIB_PARAM, payload, timeout)
        return decode_hand_set_calib_param(frame.payload)

    def get_imu_info(self, imu_index: int, timeout: float = 2.0) -> ImuInfo:
        """
        获取单个 IMU 信息。
        :param imu_index: IMU 索引
        :param timeout: 超时（秒）
        :return: IMU 信息
        """
        payload = bytes([imu_index & 0xFF, 0])
        frame = self._request(MSGCLASS_IMU, MSGID_IMU_GET_INFO, payload, timeout)
        return decode_imu_info(frame.payload)

    def get_imu_info_list(self, timeout: float = 2.0) -> ImuInfoList:
        """
        获取 IMU 列表。
        :param timeout: 超时（秒）
        :return: IMU 列表
        """
        payload = bytes([0, 0])
        frame = self._request(MSGCLASS_IMU, MSGID_IMU_GET_INFO_LIST, payload, timeout)
        return decode_imu_info_list(frame.payload)

    def imu_mag_calibrate(self, imu_index: int, action: int, timeout: float = 2.0) -> ImuMagCalibrateResult:
        """
        IMU 磁力计校准控制。
        :param imu_index: IMU 索引
        :param action: 0x01=开始，0x02=结束
        :param timeout: 超时（秒）
        :return: 校准结果
        """
        payload = bytes([action & 0xFF, imu_index & 0xFF, 0, 0])
        frame = self._request(MSGCLASS_IMU, MSGID_IMU_MAG_CALIBRATE, payload, timeout)
        return decode_imu_mag_calibrate(frame.payload)

    def set_imu_output_enable(
        self,
        enable: Union[int, bool],
        persist: Union[int, bool] = True,
        timeout: float = 2.0,
    ) -> ImuOutputEnableResult:
        """
        设置全局 IMU 运行时输出总开关。
        :param enable: 是否开启输出（0/1 或 bool）
        :param persist: 是否写入 Flash 持久化（默认 True）
        :param timeout: 超时（秒）
        :return: 输出开关结果
        """
        payload = bytes([1 if bool(enable) else 0, 1 if bool(persist) else 0, 0, 0])
        frame = self._request(MSGCLASS_IMU, MSGID_IMU_SET_OUTPUT_ENABLE, payload, timeout)
        return decode_imu_output_enable(frame.payload)

    def get_imu_output_enable(self, timeout: float = 2.0) -> ImuOutputEnableResult:
        """
        读取全局 IMU 运行时输出总开关。
        :param timeout: 超时（秒）
        :return: 输出开关结果
        """
        frame = self._request(MSGCLASS_IMU, MSGID_IMU_GET_OUTPUT_ENABLE, b"", timeout)
        return decode_imu_output_enable(frame.payload)

    def get_wireless_status_info(self, timeout: float = 2.0) -> WirelessStatusInfo:
        """
        获取 Wireless 链路/配对状态。
        :param timeout: 超时（秒）
        :return: Wireless 状态信息
        """
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_GET_STATUS_INFO, b"", timeout)
        return decode_wireless_status_info(frame.payload)

    def wireless_pair_start(
        self,
        timeout: float = 2.0,
        role_mode: Optional[int] = None,
    ) -> WirelessCommandResult:
        """
        启动 Wireless 配对流程。
        :param timeout: 超时（秒）
        :param role_mode: 配对角色模式（0=主模式, 1=从模式, 2=直接配对）。为 None 时发送空载荷，沿用固件默认角色。
        :return: 动作受理结果
        """
        if role_mode is None:
            payload = b""
        else:
            mode = int(role_mode)
            if mode not in (WIRELESS_ROLE_MASTER, WIRELESS_ROLE_SLAVE, WIRELESS_ROLE_DIRECT_PAIR):
                raise ValueError("role_mode 仅支持 0(主模式)/1(从模式)/2(直接配对)")
            payload = bytes([mode & 0xFF, 0, 0, 0])
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_PAIR_START, payload, timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.PairStart")

    def wireless_pair_cancel(self, timeout: float = 2.0) -> WirelessCommandResult:
        """
        取消 Wireless 配对流程。
        :param timeout: 超时（秒）
        :return: 动作受理结果
        """
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_PAIR_CANCEL, b"", timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.PairCancel")

    def wireless_reset(self, timeout: float = 2.0) -> WirelessCommandResult:
        """
        复位 Wireless 模块。
        :param timeout: 超时（秒）
        :return: 动作受理结果
        """
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_RESET, b"", timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.Reset")

    def wireless_enter_config(self, timeout: float = 2.0) -> WirelessCommandResult:
        """
        进入 Wireless 配置态。
        :param timeout: 超时（秒）
        :return: 动作受理结果
        """
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_ENTER_CONFIG, b"", timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.EnterConfig")

    def wireless_exit_config(self, timeout: float = 2.0) -> WirelessCommandResult:
        """
        退出 Wireless 配置态。
        :param timeout: 超时（秒）
        :return: 动作受理结果
        """
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_EXIT_CONFIG, b"", timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.ExitConfig")

    def wireless_set_stream_config(self, passthrough_enabled: Union[int, bool], timeout: float = 2.0) -> WirelessCommandResult:
        """
        设置 Wireless 透传开关。
        :param passthrough_enabled: 0/1 或 False/True
        :param timeout: 超时（秒）
        :return: 设置结果
        """
        value = int(passthrough_enabled)
        if value not in (0, 1):
            raise ValueError("passthrough_enabled 仅支持 0 或 1")
        payload = bytes([value, 0, 0, 0])
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_SET_STREAM_CONFIG, payload, timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.SetStreamConfig")

    def wireless_set_push_freq(self, push_freq_option: int, timeout: float = 2.0) -> WirelessCommandResult:
        """
        设置 Wireless 推送频率档位。
        当前固件支持 2/4，对应约 2/4ms。
        :param push_freq_option: 推送档位
        :param timeout: 超时（秒）
        :return: 设置结果
        """
        option = int(push_freq_option)
        if option not in (2, 4):
            raise ValueError("push_freq_option 仅支持 2 或 4（对应约 2/4ms）")
        payload = bytes([option & 0xFF])
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_SET_PUSH_FREQ, payload, timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.SetPushFreq")

    def wireless_set_pair_result_info(
        self,
        pair_result: int,
        paired_device_name: Union[str, bytes],
        timeout: float = 2.0,
    ) -> WirelessCommandResult:
        """
        写入 Wireless 配对结果与对端设备名。
        :param pair_result: 配对结果（0~5）
        :param paired_device_name: 对端设备名称
        :param timeout: 超时（秒）
        :return: 设置结果
        """
        if not 0 <= int(pair_result) <= WIRELESS_PAIR_RESULT_ERROR:
            raise ValueError("pair_result 仅支持 0~5")
        payload = (
            bytes([pair_result & 0xFF, 0, 0, 0]) +
            _encode_fixed_ascii_string(paired_device_name, WIRELESS_DEVICE_NAME_LEN)
        )
        frame = self._request(MSGCLASS_WIRELESS, MSGID_WIRELESS_SET_PAIR_RESULT_INFO, payload, timeout)
        return decode_wireless_command_result(frame.payload, "Wireless.SetPairResultInfo")

    def haptics_set_output(
        self,
        channel_id: int,
        amplitude: int,
        pattern_id: int,
        duration_ms: int,
        timeout: float = 2.0,
    ) -> HapticsOutputResult:
        """
        设置振动输出。
        :param channel_id: 通道号
        :param amplitude: 幅度 0..100
        :param pattern_id: 模式 ID
        :param duration_ms: 持续时间
        :param timeout: 超时（秒）
        :return: 设置结果
        """
        payload = bytes([0, channel_id & 0xFF, amplitude & 0xFF, pattern_id & 0xFF]) + struct.pack('<H', duration_ms & 0xFFFF)
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_SET_OUTPUT, payload, timeout)
        return decode_haptics_output_result(frame.payload)

    def haptics_drv_get_cal_status(self, handset_index: int, timeout: float = 2.0) -> HapticsDrvCalStatus:
        """
        获取驱动校准状态。
        """
        payload = bytes([handset_index & 0xFF, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_DRV_GET_CAL_STATUS, payload, timeout)
        return decode_haptics_drv_cal_status(frame.payload)

    def haptics_drv_calibrate(self, handset_index: int, timeout: float = 2.0) -> HapticsDrvCalibrateResult:
        """
        触发驱动校准。
        """
        payload = bytes([handset_index & 0xFF, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_DRV_CALIBRATE, payload, timeout)
        return decode_haptics_drv_calibrate(frame.payload)

    def haptics_vibrate_play(self, handset_index: int, effect_id: int, timeout: float = 2.0) -> HapticsVibratePlayResult:
        """
        播放预置振动效果。
        """
        payload = bytes([handset_index & 0xFF, effect_id & 0xFF, 0, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_VIBRATE_PLAY, payload, timeout)
        return decode_haptics_vibrate_play(frame.payload)

    def haptics_vibrate_stop(self, handset_index: int, timeout: float = 2.0) -> HapticsVibrateStopResult:
        """
        停止振动。
        """
        payload = bytes([handset_index & 0xFF, 0, 0, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_VIBRATE_STOP, payload, timeout)
        return decode_haptics_vibrate_stop(frame.payload)

    def haptics_vibrate_realtime(self, handset_index: int, amplitude: int, timeout: float = 2.0) -> HapticsVibrateRealtimeResult:
        """
        实时振动。
        """
        payload = bytes([handset_index & 0xFF, amplitude & 0xFF, 0, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_VIBRATE_REALTIME, payload, timeout)
        return decode_haptics_vibrate_realtime(frame.payload)

    def haptics_set_enable(self, handset_index: int, enable: Union[int, bool], timeout: float = 2.0) -> HapticsEnableResult:
        """
        设置使能。
        """
        payload = bytes([handset_index & 0xFF, 1 if bool(enable) else 0, 0, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_SET_ENABLE, payload, timeout)
        return decode_haptics_enable(frame.payload)

    def haptics_set_mode(self, handset_index: int, mode: int, timeout: float = 2.0) -> HapticsModeResult:
        """
        设置模式。
        """
        payload = bytes([handset_index & 0xFF, mode & 0xFF, 0, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_SET_MODE, payload, timeout)
        return decode_haptics_mode(frame.payload)

    def haptics_set_pressure(self, handset_index: int, pressure: int, timeout: float = 2.0) -> HapticsPressureResult:
        """
        设置压力值。
        """
        payload = bytes([handset_index & 0xFF, 0]) + struct.pack('<H', pressure & 0xFFFF)
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_SET_PRESSURE, payload, timeout)
        return decode_haptics_pressure(frame.payload)

    def haptics_set_timeout(self, handset_index: int, timeout_ms: int, timeout: float = 2.0) -> HapticsTimeoutResult:
        """
        设置超时值。
        """
        payload = bytes([handset_index & 0xFF, 0]) + struct.pack('<H', timeout_ms & 0xFFFF)
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_SET_TIMEOUT, payload, timeout)
        return decode_haptics_timeout(frame.payload)

    def haptics_get_intensity(self, handset_index: int, timeout: float = 2.0) -> HapticsIntensityResult:
        """
        获取强度。
        """
        payload = bytes([handset_index & 0xFF, 0])
        frame = self._request(MSGCLASS_HAPTICS, MSGID_HAPTICS_GET_INTENSITY, payload, timeout)
        return decode_haptics_intensity(frame.payload)

    def wait_wireless_pair_result(
        self,
        timeout: float = 35.0,
        poll_interval: float = 0.3,
        request_timeout: float = 2.0,
    ) -> WirelessStatusInfo:
        """
        轮询 Wireless 状态，直到配对流程进入终态。
        :param timeout: 总等待超时（秒）
        :param poll_interval: 轮询间隔（秒）
        :param request_timeout: 单次 QnTP 请求超时（秒）
        :return: 最终状态
        """
        if timeout <= 0:
            raise ValueError("timeout 必须大于 0")
        if poll_interval <= 0:
            raise ValueError("poll_interval 必须大于 0")
        if request_timeout <= 0:
            raise ValueError("request_timeout 必须大于 0")

        deadline = time.time() + timeout
        last_status: Optional[WirelessStatusInfo] = None

        while time.time() < deadline:
            last_status = self.get_wireless_status_info(timeout=request_timeout)
            if (
                last_status.pairing_busy == 0 and
                last_status.pair_result in (
                    WIRELESS_PAIR_RESULT_OK,
                    WIRELESS_PAIR_RESULT_TIMEOUT,
                    WIRELESS_PAIR_RESULT_CANCELED,
                    WIRELESS_PAIR_RESULT_ERROR,
                )
            ):
                return last_status
            time.sleep(poll_interval)

        if last_status is not None:
            raise QnTPTimeoutError(
                "等待 Wireless 配对结果超时 "
                f"(busy={last_status.pairing_busy}, step={last_status.pair_step}, result={last_status.pair_result})"
            )
        raise QnTPTimeoutError("等待 Wireless 配对结果超时")

    def wireless_pair_start_and_wait(
        self,
        role_mode: Optional[int] = None,
        timeout: float = 35.0,
        poll_interval: float = 0.3,
        request_timeout: float = 2.0,
    ) -> WirelessPairWaitResult:
        """
        启动 Wireless 配对并等待终态。
        :param role_mode: 配对角色模式（0=主模式, 1=从模式, 2=直接配对）
        :param timeout: 总等待超时（秒）
        :param poll_interval: 状态轮询间隔（秒）
        :param request_timeout: 单次 QnTP 请求超时（秒）
        :return: 配对等待结果
        """
        start_time = time.time()
        accepted = self.wireless_pair_start(timeout=request_timeout, role_mode=role_mode)
        final_status = self.wait_wireless_pair_result(
            timeout=timeout,
            poll_interval=poll_interval,
            request_timeout=request_timeout,
        )
        elapsed_s = time.time() - start_time
        return WirelessPairWaitResult(
            accepted_status=accepted.status_code,
            final_status=final_status,
            elapsed_s=elapsed_s,
        )

    # ---- 内部实现 ----

    def _next_seq(self) -> int:
        """
        生成下一个序号。
        :return: 0-255 的序号
        """
        with self._pending_lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
            return seq

    def _pop_late_response(self, msg_class: int, msg_id: int, within: float) -> Optional[QnTPFrame]:
        """
        从迟到缓存中提取匹配响应。
        :param msg_class: 功能域
        :param msg_id: 消息 ID
        :param within: 允许的时间窗口（秒）
        :return: 匹配的响应或 None
        """
        """从迟到缓存中提取匹配响应（CLI 体验优化）"""
        with self._late_lock:
            now = time.time()
            # 先清理过期
            self._late_responses = [
                (t, f) for (t, f) in self._late_responses if now - t <= within
            ]
            for i, (t, f) in enumerate(self._late_responses):
                if f.msg_class == msg_class and f.msg_id == msg_id:
                    self._late_responses.pop(i)
                    return f
        return None

    def _request(self, msg_class: int, msg_id: int, payload: bytes, timeout: float) -> QnTPFrame:
        """
        发送请求并等待响应。
        :param msg_class: 功能域
        :param msg_id: 消息 ID
        :param payload: 请求载荷
        :param timeout: 超时（秒）
        :return: 响应帧
        """
        if not self._serial:
            raise RuntimeError("串口未启动")
        seq = self._next_seq()
        pending = _PendingRequest(msg_class, msg_id)
        with self._pending_lock:
            self._pending[seq] = pending
        frame = build_qntp_frame(QNTP_MSG_REQUEST, msg_class, msg_id, seq, payload)
        with self._write_lock:
            self._serial.write(frame)
        if not pending.event.wait(timeout):
            with self._pending_lock:
                self._pending.pop(seq, None)
            # 允许短时间内“迟到”的响应被复用
            late = self._pop_late_response(msg_class, msg_id, within=0.8)
            if late is not None:
                return late
            raise QnTPTimeoutError(f"等待响应超时（class=0x{msg_class:02X}, id=0x{msg_id:02X}）")
        if pending.response is None:
            raise QnTPTimeoutError("响应丢失")
        return pending.response

    def _enqueue_event(self, kind: str, payload: Union[TelemetrySnapshot, QnTPFrame]) -> None:
        try:
            self._dispatch_queue.put_nowait((kind, payload))
        except queue.Full:
            self._dispatch_dropped += 1

    def _dispatch_loop(self) -> None:
        """
        回调分发线程，避免回调阻塞接收线程。
        """
        while self._running or not self._dispatch_queue.empty():
            try:
                kind, payload = self._dispatch_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if kind == "telemetry":
                for cb in list(self._telemetry_callbacks):
                    try:
                        cb(payload)  # type: ignore[arg-type]
                    except Exception:
                        pass
            elif kind == "qntp":
                for cb in list(self._qntp_callbacks):
                    try:
                        cb(payload)  # type: ignore[arg-type]
                    except Exception:
                        pass
            elif kind == "notify":
                for cb in list(self._notify_callbacks):
                    try:
                        cb(payload)  # type: ignore[arg-type]
                    except Exception:
                        pass

    def _read_loop(self) -> None:
        """
        串口接收线程主循环。
        :return: None
        """
        while self._running and self._serial:
            try:
                # macOS 上 in_waiting 可能长期为 0，使用短阻塞读兜底
                to_read = None
                if hasattr(self._serial, "in_waiting"):
                    try:
                        waiting = int(self._serial.in_waiting)
                    except Exception:
                        waiting = 0
                    if waiting > 0:
                        to_read = min(waiting, self.read_chunk_size)
                    else:
                        # 读固定块，避免每次只读 1 字节导致吞吐过低
                        to_read = self.read_chunk_size
                if to_read is None:
                    to_read = self.read_chunk_size
                chunk = self._serial.read(to_read)
                if not chunk:
                    continue
                self._bytes_read += len(chunk)
                items = self._parser.feed(chunk)
                for item in items:
                    if isinstance(item, TelemetrySnapshot):
                        self._latest_telemetry = item
                        self._enqueue_event("telemetry", item)
                        continue

                    # QnTP 帧
                    frame = item
                    self._enqueue_event("qntp", frame)

                    if frame.msg_type == QNTP_MSG_RESPONSE:
                        with self._pending_lock:
                            pending = self._pending.pop(frame.seq, None)
                            # 兼容固件：若 Seq 不匹配，但仅有一个同类请求在等待，尝试匹配
                            if pending is None:
                                candidates = [
                                    (k, v) for k, v in self._pending.items()
                                    if isinstance(v, _PendingRequest)
                                    and v.msg_class == frame.msg_class
                                    and v.msg_id == frame.msg_id
                                ]
                                if len(candidates) == 1:
                                    alt_seq, alt_pending = candidates[0]
                                    pending = self._pending.pop(alt_seq, None)
                        if pending:
                            pending.response = frame
                            pending.event.set()
                        else:
                            # 可能已超时但响应晚到，放入短暂缓存
                            with self._late_lock:
                                now = time.time()
                                self._late_responses.append((now, frame))
                                # 清理过期缓存
                                self._late_responses = [
                                    (t, f) for (t, f) in self._late_responses if now - t <= 1.0
                                ]
                        continue

                    if frame.msg_type == QNTP_MSG_NOTIFY:
                        self._enqueue_event("notify", frame)

            except Exception:
                # 避免后台线程硬崩溃
                time.sleep(0.01)


# -----------------------------
