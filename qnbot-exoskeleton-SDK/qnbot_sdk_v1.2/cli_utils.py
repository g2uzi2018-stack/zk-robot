#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Qnbot SDK CLI 工具（交互与可视化输出）。

修订版本：V1.2
修订日期：20260306
作者：杭州启能机器人有限公司

Qnbot SDK 使用示例。
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional
import json
import locale
import os
import select
import sys
import threading
import time

from qnbot_sdk import (
    BATTERY_PERCENT_CHARGING,
    QnbotClient,
    QnTPFrame,
    TelemetrySnapshot,
    DeviceProfileTopology,
    DeviceProfileDataProfiles,
    TelemetryStreamRuntimeConfigResult,
    SystemCapabilities,
    SystemStatusInfo,
    SystemVersionInfo,
    EncoderInfo,
    EncoderInfoList,
    EncoderZeroValue,
    EncoderSetZeroHereResult,
    EncoderSetZeroValueResult,
    HandInfo,
    HandInfoList,
    HandCalibParams,
    HandCommandResult,
    HandCalibCommitResult,
    HandSetCalibOutputResult,
    HandSetCalibParamResult,
    ImuInfo,
    ImuInfoList,
    ImuMagCalibrateResult,
    ImuOutputEnableResult,
    WirelessCommandResult,
    WirelessPairWaitResult,
    WirelessStatusInfo,
    HapticsOutputResult,
    HapticsDrvCalStatus,
    HapticsDrvCalibrateResult,
    HapticsVibratePlayResult,
    HapticsVibrateStopResult,
    HapticsVibrateRealtimeResult,
    HapticsEnableResult,
    HapticsModeResult,
    HapticsPressureResult,
    HapticsTimeoutResult,
    HapticsIntensityResult,
    WIRELESS_PAIR_RESULT_CANCELED,
    WIRELESS_PAIR_RESULT_ERROR,
    WIRELESS_PAIR_RESULT_NONE,
    WIRELESS_PAIR_RESULT_OK,
    WIRELESS_PAIR_RESULT_RUNNING,
    WIRELESS_PAIR_RESULT_TIMEOUT,
    WIRELESS_ROLE_DIRECT_PAIR,
    WIRELESS_ROLE_MASTER,
    WIRELESS_ROLE_SLAVE,
    TELEMETRY_STREAM_ID_HIGH_RATE_SNAPSHOT,
    TELEMETRY_STREAM_ID_LEGACY_PROTOCOL,
    HAND_CALIB_POINT_JX_CENTER,
    HAND_CALIB_POINT_JX_MAX,
    HAND_CALIB_POINT_JX_MIN,
    HAND_CALIB_POINT_JY_CENTER,
    HAND_CALIB_POINT_JY_MAX,
    HAND_CALIB_POINT_JY_MIN,
    HAND_CALIB_POINT_TRIG_START,
    HAND_CALIB_POINT_TRIG_MAX,
    MSGCLASS_DEVICE_PROFILE,
    MSGCLASS_TELEMETRY,
    MSGCLASS_HANDSET,
    MSGCLASS_HAPTICS,
    MSGCLASS_IMU,
    MSGCLASS_WIRELESS,
)


# -----------------------------
# CLI 工具函数
# -----------------------------

DEVICE_TYPE_EXO_STD = 0x0101
DEVICE_TYPE_EXO_PLUS = 0x0102
DEVICE_TYPE_EXO_PLUS_WIRELESS = 0x0103

_DEVICE_TYPE_NAME = {
    DEVICE_TYPE_EXO_STD: "Qnbot Exo Std",
    DEVICE_TYPE_EXO_PLUS: "Qnbot Exo Plus",
    DEVICE_TYPE_EXO_PLUS_WIRELESS: "Qnbot Exo Plus Wireless / RF",
}


# -----------------------------
# i18n (CLI runtime localization)
# -----------------------------

_CLI_LANG = "zh-CN"
_CLI_REPLACEMENTS: List[tuple[str, str]] = []
_BUILTIN_PRINT = print


def _detect_cli_lang() -> str:
    # Detection order:
    # 1) explicit locale environment variables
    # 2) Python runtime locale on current OS
    # 3) default to zh-CN to preserve existing user experience
    candidates = [
        os.environ.get("QNBOT_CLI_LANG", ""),
        os.environ.get("LC_ALL", ""),
        os.environ.get("LC_MESSAGES", ""),
        os.environ.get("LANG", ""),
    ]
    try:
        loc, _ = locale.getdefaultlocale()
        if loc:
            candidates.append(loc)
    except Exception:
        pass
    try:
        loc2, _ = locale.getlocale()
        if loc2:
            candidates.append(loc2)
    except Exception:
        pass

    merged = " ".join(candidates).lower()
    if "en" in merged:
        return "en-US"
    if "zh" in merged or "chinese" in merged:
        return "zh-CN"
    return "zh-CN"


def _load_locale_replacements(lang: str) -> List[tuple[str, str]]:
    if lang == "zh-CN":
        return []
    locale_path = os.path.join(os.path.dirname(__file__), "locales", f"{lang}.json")
    try:
        with open(locale_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return []
    mapping = data.get("replacements", {}) if isinstance(data, dict) else {}
    if not isinstance(mapping, dict):
        return []
    # Longer phrases first to avoid partial replacement collisions.
    ordered = sorted(
        ((str(k), str(v)) for k, v in mapping.items()),
        key=lambda x: len(x[0]),
        reverse=True,
    )
    return ordered


def _set_cli_language(lang: str) -> None:
    global _CLI_LANG, _CLI_REPLACEMENTS
    if lang == "auto":
        lang = _detect_cli_lang()
    if lang not in ("zh-CN", "en-US"):
        lang = "zh-CN"
    _CLI_LANG = lang
    _CLI_REPLACEMENTS = _load_locale_replacements(lang)


def _normalize_cli_lang_token(token: str) -> Optional[str]:
    text = token.strip()
    low = text.lower()
    if low == "auto":
        return "auto"
    if low in ("zh", "zh-cn", "zh_cn", "cn", "chinese"):
        return "zh-CN"
    if low in ("en", "en-us", "en_us", "english"):
        return "en-US"
    return None


def _tr(text: str) -> str:
    if _CLI_LANG == "zh-CN" or not text or not _CLI_REPLACEMENTS:
        return text
    out = text
    for src, dst in _CLI_REPLACEMENTS:
        if src in out:
            out = out.replace(src, dst)
    return out


def _localized_print(*args, **kwargs) -> None:
    if _CLI_LANG == "zh-CN":
        _BUILTIN_PRINT(*args, **kwargs)
        return
    loc_args = [(_tr(a) if isinstance(a, str) else a) for a in args]
    _BUILTIN_PRINT(*loc_args, **kwargs)


def _lwrite(text: str) -> None:
    if isinstance(text, str):
        sys.stdout.write(_tr(text))
    else:
        sys.stdout.write(text)


def _runtime_unsupported_message(command_plane: str) -> str:
    if _CLI_LANG == "en-US":
        mapping = {
            "wl.*": "Current device does not support wl.* commands (use help to view available commands).",
            "imu.output.*": "Current device does not support imu.output.* commands (use help to view available commands).",
            "haptics.*": "Current device does not support haptics.* commands (use help to view available commands).",
        }
        return mapping.get(command_plane, f"Current device does not support {command_plane} commands (use help to view available commands).")
    mapping = {
        "wl.*": "当前设备不支持 wl.* 命令（请使用 help 查看可用命令）。",
        "imu.output.*": "当前设备不支持 imu.output.* 命令（请使用 help 查看可用命令）。",
        "haptics.*": "当前设备不支持 haptics.* 命令（请使用 help 查看可用命令）。",
    }
    return mapping.get(command_plane, f"当前设备不支持 {command_plane} 命令（请使用 help 查看可用命令）。")


# Route all module-level print calls through localization.
print = _localized_print


def _strip_placeholder(value: str) -> str:
    text = value.strip()
    if len(text) >= 2 and ((text[0] == "[" and text[-1] == "]") or
                           (text[0] == "<" and text[-1] == ">")):
        text = text[1:-1].strip()
    return text


def _parse_int(value: str) -> int:
    """支持十进制与 0x 开头的十六进制"""
    return int(_strip_placeholder(value), 0)


def _caps_has_class(caps: Optional[SystemCapabilities], msg_class: int) -> bool:
    if caps is None:
        return False
    if caps.status_code != 0x00:
        return False
    if len(caps.supported_class) < 32:
        return False
    byte_idx = msg_class >> 3
    bit_idx = msg_class & 0x07
    if byte_idx >= len(caps.supported_class):
        return False
    return ((caps.supported_class[byte_idx] >> bit_idx) & 0x01) == 1


def _build_runtime_support(
    device_type: Optional[int],
    caps: Optional[SystemCapabilities],
) -> Dict[str, bool]:
    support = {
        "device_profile": True,
        "telemetry_runtime": True,
        "hand": True,
        "haptics": True,
        "imu_output": True,
        "wireless": True,
    }
    if device_type is None:
        return support
    support["device_profile"] = _caps_has_class(caps, MSGCLASS_DEVICE_PROFILE)
    support["telemetry_runtime"] = _caps_has_class(caps, MSGCLASS_TELEMETRY)
    support["hand"] = _caps_has_class(caps, MSGCLASS_HANDSET)
    support["haptics"] = _caps_has_class(caps, MSGCLASS_HAPTICS)
    # IMU 输出总开关按能力位图启用（std/plus/rf 均可能支持，最终以固件实现为准）。
    support["imu_output"] = _caps_has_class(caps, MSGCLASS_IMU)
    # Wireless 命令面优先看能力位图，未置位则隐藏。
    support["wireless"] = _caps_has_class(caps, MSGCLASS_WIRELESS)
    return support


def _print_help(runtime_support: Optional[Dict[str, bool]] = None) -> None:
    def _enabled(key: str) -> bool:
        if runtime_support is None:
            return True
        return runtime_support.get(key, True)

    print("可用命令：")
    print("  help                               显示帮助")
    print("  q                                  直接退出（无需回车）")
    print("  exit | quit                        退出")
    print("  sys.version                        获取系统版本")
    print("  sys.status                         获取系统状态（电池/无线/SN/设备名）")
    print("  sys.caps                           获取系统能力位图")
    print("  sys.dfu                            请求进入系统 DFU")
    print("  telemetry on|off                   开关遥测打印")
    print("  telemetry mode line|stream         遥测输出模式（行刷新/持续输出）")
    if _enabled("telemetry_runtime"):
        print("  tm.stream <stream_id> <0|1>        设置 Telemetry 运行时流开关，stream_id=0xF1/0x00")
    if _enabled("device_profile"):
        print("  dev.topo                           获取设备拓扑信息")
        print("  dev.profiles                       获取设备数据 profile（当前固件通常返回 UNSUPPORTED）")
    print("  enc.list                           获取编码器列表")
    print("  enc.info <ch>                      获取单个编码器信息")
    print("  enc.zero.get <ch>                  获取编码器零位")
    print("  enc.zero.set_here <ch>             当前位置设为零位")
    print("  enc.zero.set <ch> <zero_value>     设置编码器零位值")
    if _enabled("hand"):
        print("  hand.list                          获取手柄列表")
        print("  hand.info <idx>                    获取单个手柄信息")
        print("  hand.calib.get <idx>               获取手柄校准参数")
        print("  hand.calib.start <idx>             开始手柄校准")
        print("  hand.calib.commit <idx> <point>    提交手柄校准点")
        print("  hand.calib.finish <idx>            结束手柄校准")
        print("  hand.calib.wizard [idx]            启动引导式手柄校准流程")
        print("  hand.out.get <idx>                 获取手柄校准输出状态")
        print("  hand.out.set <idx> <0|1>           设置手柄校准输出状态")
        print("  hand.param.set <idx> <point> <v>   设置手柄校准参数值")
    print("  imu.list                           获取 IMU 列表")
    print("  imu.info <idx>                     获取单个 IMU 信息")
    print("  imu.mag.start <idx>                开始 IMU 磁力计校准")
    print("  imu.mag.finish <idx>               结束 IMU 磁力计校准")
    print("  imu.mag.wizard [idx]               启动引导式 IMU 磁力计校准")
    if _enabled("imu_output"):
        print("  imu.output.get                     获取全局 IMU 输出总开关状态")
        print("  imu.output.set <0|1> [persist]     设置全局 IMU 输出总开关，persist=0/1，默认 1")
    if _enabled("wireless"):
        print("  wl.status                          获取 Wireless 状态")
        print("  wl.pair.start [role]               启动 Wireless 配对，role=master/slave/direct")
        print("  wl.pair.wait [timeout_s]           等待配对终态")
        print("  wl.pair.run [role] [timeout_s]     启动配对并等待终态")
        print("  wl.pair.wizard [role] [timeout_s]  启动引导式 Wireless 配对流程")
        print("  wl.pair.cancel                     取消 Wireless 配对")
        print("  wl.reset                           复位 Wireless 模块")
        print("  wl.config.enter                    进入 Wireless 配置态")
        print("  wl.config.exit                     退出 Wireless 配置态")
        print("  wl.stream <enable>                 设置 Wireless 透传开关，enable=0/1")
        print("  wl.push <freq>                     设置 Wireless 推送频率，freq=2/4")
        print("  wl.result <0..5> [peer_name]       写入配对结果与对端设备名")
    # Haptics commands stay visible in help even when the capability bit is absent.
    # Runtime execution still checks support and returns a clear unsupported message.
    print("  haptics.out <ch> <amp> <pat> <ms>  设置 Haptics 输出")
    print("  haptics.cal.get <idx>              获取 Haptics 驱动校准状态")
    print("  haptics.cal.run <idx>              触发 Haptics 驱动校准")
    print("  haptics.play <idx> <effect>        播放 Haptics 预置效果")
    print("  haptics.stop <idx>                 停止 Haptics 输出")
    print("  haptics.rt <idx> <amp>             设置 Haptics 实时振幅")
    print("  haptics.enable <idx> <0|1>         设置 Haptics 使能")
    print("  haptics.mode <idx> <mode>          设置 Haptics 模式")
    print("  haptics.pressure <idx> <value>     设置 Haptics 压力值")
    print("  haptics.timeout <idx> <ms>         设置 Haptics 超时值")
    print("  haptics.intensity <idx>            获取 Haptics 强度")
    print("  haptics.wizard                     启动 Haptics 引导调试流程")
    print("  haptics.force.test <idx> <pressure> [mode] [timeout_ms] [hold_s]")
    print("                                     力反馈定压测试（按 q 停止）")
    print("  haptics.trigger.sim <idx> [threshold] [poll_ms]")
    print("                                     扳机震感模拟（按 q 停止）")
    print("  stats                              显示解析统计信息")
    print("  lang <auto|zh-CN|en-US>            运行中切换 CLI 语言 (Switch CLI language at runtime)")
    if runtime_support is not None and not _enabled("imu_output"):
        print(_tr("提示：当前设备未开放 IMU 输出总开关命令（imu.output.*）。"))
    if runtime_support is not None and not _enabled("wireless"):
        print(_tr("提示：当前设备未开放 Wireless 命令面（wl.*）。"))
    if _enabled("wireless"):
        print("示例：wl.pair.start slave | wl.stream 1 | wl.push 4（帮助中的 []/<> 不需要输入）")
    else:
        print("示例：enc.list | imu.list | stats（帮助中的 []/<> 不需要输入）")


def _print_system_info(info: SystemVersionInfo) -> None:
    print(
        "系统版本(System.GetVersion):",
        f"状态=0x{info.status_code:02X}",
        f"协议={info.proto_major}.{info.proto_minor}",
        f"设备=0x{info.device_type:04X}",
        f"平台={info.platform}",
        f"修订={info.revision}",
        f"特性={info.feature}",
        f"构建={info.build}",
    )


def _print_system_status(info: SystemStatusInfo) -> None:
    serial_text = info.serial.decode("ascii", errors="ignore") if info.serial else "-"
    device_name_text = info.device_name if info.device_name else "-"
    if info.battery_percent == BATTERY_PERCENT_CHARGING:
        battery_text = f"{info.battery_mv}mV/充电中"
    else:
        battery_text = f"{info.battery_mv}mV/{info.battery_percent}%"
    print(
        "系统状态(System.Status):",
        f"状态=0x{info.status_code:02X}",
        f"电池={battery_text}",
        f"无线={'已连接' if info.wireless_link_state else '未连接'}",
        f"SN={serial_text}",
        f"Name={device_name_text}",
    )


def _print_caps(caps: SystemCapabilities) -> None:
    print(f"系统能力(System.GetCapabilities): 状态=0x{caps.status_code:02X}")
    print(f"支持位图(32B)={caps.supported_class.hex()}")


def _print_device_topology(info: DeviceProfileTopology) -> None:
    print(
        "设备拓扑(DeviceProfile.GetTopology):",
        f"status=0x{info.status_code:02X}",
        f"enc={info.encoder_count}/{info.encoder_total_slots}",
        f"hand={info.handset_count}/{info.handset_total_slots}",
        f"imu={info.imu_count}/{info.imu_total_slots}",
    )


def _print_device_profiles(info: DeviceProfileDataProfiles) -> None:
    print(
        "设备数据Profile(DeviceProfile.GetDataProfiles):",
        f"status=0x{info.status_code:02X}({_status_text(info.status_code)})",
        f"raw={info.raw_payload.hex() if info.raw_payload else '-'}",
    )


def _print_telemetry_runtime_result(result: TelemetryStreamRuntimeConfigResult) -> None:
    stream_name = {
        TELEMETRY_STREAM_ID_LEGACY_PROTOCOL: "LegacyProtocol",
        TELEMETRY_STREAM_ID_HIGH_RATE_SNAPSHOT: "HighRateSnapshot",
    }.get(result.stream_id, f"0x{result.stream_id:02X}")
    print(
        "Telemetry.SetStreamRuntimeConfig:",
        f"status=0x{result.status_code:02X}({_status_text(result.status_code)})",
        f"stream={stream_name}",
    )


def _print_encoder_info(info: EncoderInfo) -> None:
    if info.status_code != 0x00:
        print(f"编码器信息: status=0x{info.status_code:02X}（设备可能不支持）")
        return
    print(
        f"编码器信息: status=0x{info.status_code:02X} "
        f"ch={info.channel_index} id={info.encoder_id} "
        f"hw=0x{info.hw_version:04X} fw=0x{info.fw_version:04X} "
        f"serial={info.serial.hex()}"
    )


def _print_encoder_list(info_list: EncoderInfoList) -> None:
    if info_list.status_code != 0x00:
        print(f"编码器列表: status=0x{info_list.status_code:02X}（设备可能不支持）")
        return
    print(f"编码器列表: status=0x{info_list.status_code:02X} count={len(info_list.entries)}")
    for e in info_list.entries:
        print(
            f"  ch={e.channel_index} id={e.encoder_id} "
            f"hw=0x{e.hw_version:04X} fw=0x{e.fw_version:04X} "
            f"serial={e.serial.hex()}"
        )


def _print_hand_info(info: HandInfo) -> None:
    if info.status_code != 0x00:
        print(f"手柄信息: status=0x{info.status_code:02X}（设备可能不支持）")
        return
    print(
        f"手柄信息: status=0x{info.status_code:02X} "
        f"idx={info.handset_index} hw=0x{info.hw_version:04X} "
        f"fw=0x{info.fw_version:04X} serial={info.serial.hex()}"
    )


def _print_hand_list(info_list: HandInfoList) -> None:
    if info_list.status_code != 0x00:
        print(f"手柄列表: status=0x{info_list.status_code:02X}（设备可能不支持）")
        return
    print(f"手柄列表: status=0x{info_list.status_code:02X} count={len(info_list.entries)}")
    for e in info_list.entries:
        print(
            f"  idx={e.handset_index} hw=0x{e.hw_version:04X} "
            f"fw=0x{e.fw_version:04X} serial={e.serial.hex()}"
        )


def _print_hand_calib_params(info: HandCalibParams) -> None:
    print(
        "手柄校准参数:",
        f"status=0x{info.status_code:02X}({_status_text(info.status_code)})",
        f"idx={info.handset_index}",
        f"out={info.calib_output_enabled}",
        f"jx=({info.jx_center},{info.jx_max},{info.jx_min})",
        f"jy=({info.jy_center},{info.jy_max},{info.jy_min})",
        f"trig=({info.trig_start},{info.trig_max})",
    )


def _print_hand_command_result(label: str, result: HandCommandResult) -> None:
    print(
        f"{label}: status=0x{result.status_code:02X}({_status_text(result.status_code)}) "
        f"idx={result.handset_index}"
    )


def _hand_calib_point_desc(point: int) -> str:
    mapping = {
        HAND_CALIB_POINT_JX_CENTER: "JX_CENTER(摇杆X中心)",
        HAND_CALIB_POINT_JX_MAX: "JX_MAX(摇杆X最大/左侧)",
        HAND_CALIB_POINT_JX_MIN: "JX_MIN(摇杆X最小/右侧)",
        HAND_CALIB_POINT_JY_CENTER: "JY_CENTER(摇杆Y中心)",
        HAND_CALIB_POINT_JY_MAX: "JY_MAX(摇杆Y最大/上侧)",
        HAND_CALIB_POINT_JY_MIN: "JY_MIN(摇杆Y最小/下侧)",
        HAND_CALIB_POINT_TRIG_START: "TRIG_START(扳机起始)",
        HAND_CALIB_POINT_TRIG_MAX: "TRIG_MAX(扳机最大)",
    }
    return mapping.get(point, f"UNKNOWN({point})")


def _print_hand_commit_result(result: HandCalibCommitResult) -> None:
    print(
        f"Hand.CalibCommit: status=0x{result.status_code:02X}({_status_text(result.status_code)}) "
        f"idx={result.handset_index} point={result.calib_point}({_hand_calib_point_desc(result.calib_point)})"
    )


def _print_hand_set_output_result(result: HandSetCalibOutputResult) -> None:
    print(
        f"Hand.SetCalibOutput: status=0x{result.status_code:02X}({_status_text(result.status_code)}) "
        f"idx={result.handset_index} enable={result.enable}"
    )


def _print_hand_set_param_result(result: HandSetCalibParamResult) -> None:
    print(
        f"Hand.SetCalibParam: status=0x{result.status_code:02X}({_status_text(result.status_code)}) "
        f"idx={result.handset_index} point={result.calib_point}({_hand_calib_point_desc(result.calib_point)})"
    )


def _print_imu_info(info: ImuInfo) -> None:
    if info.status_code != 0x00:
        print(f"IMU 信息: status=0x{info.status_code:02X}（设备可能不支持）")
        return
    print(
        f"IMU 信息: status=0x{info.status_code:02X} "
        f"idx={info.imu_index} version=0x{info.version:08X}"
    )


def _print_imu_list(info_list: ImuInfoList) -> None:
    if info_list.status_code != 0x00:
        print(f"IMU 列表: status=0x{info_list.status_code:02X}（设备可能不支持）")
        return
    print(f"IMU 列表: status=0x{info_list.status_code:02X} count={len(info_list.entries)}")
    for e in info_list.entries:
        print(f"  idx={e.imu_index} version=0x{e.version:08X}")


def _print_imu_output_enable(label: str, result: ImuOutputEnableResult) -> None:
    print(
        f"{label}: status=0x{result.status_code:02X}({_status_text(result.status_code)}) "
        f"enable={result.enable} persisted={result.persisted} "
        f"capability_mask=0x{result.capability_mask:02X} "
        f"err={result.error_code}"
    )


def _pair_result_text(value: int) -> str:
    mapping = {
        WIRELESS_PAIR_RESULT_NONE: "NONE",
        WIRELESS_PAIR_RESULT_RUNNING: "RUNNING",
        WIRELESS_PAIR_RESULT_OK: "OK",
        WIRELESS_PAIR_RESULT_TIMEOUT: "TIMEOUT",
        WIRELESS_PAIR_RESULT_CANCELED: "CANCELED",
        WIRELESS_PAIR_RESULT_ERROR: "ERROR",
    }
    return mapping.get(value, f"UNKNOWN({value})")


def _pair_role_value(value: str) -> int:
    role = _strip_placeholder(value).lower()
    mapping = {
        "master": WIRELESS_ROLE_MASTER,
        "m": WIRELESS_ROLE_MASTER,
        "slave": WIRELESS_ROLE_SLAVE,
        "s": WIRELESS_ROLE_SLAVE,
        "direct": WIRELESS_ROLE_DIRECT_PAIR,
        "d": WIRELESS_ROLE_DIRECT_PAIR,
        "directpair": WIRELESS_ROLE_DIRECT_PAIR,
    }
    if role not in mapping:
        raise ValueError("role 仅支持 master/slave/direct")
    return mapping[role]


def _status_text(value: int) -> str:
    mapping = {
        0x00: "OK",
        0x01: "BAD_PARAM",
        0x02: "BUSY",
        0x03: "NOT_READY",
        0x04: "UNSUPPORTED",
    }
    return mapping.get(value, f"UNKNOWN({value})")


def _parse_stream_id(value: str) -> int:
    stream = _parse_int(value)
    if stream not in (TELEMETRY_STREAM_ID_LEGACY_PROTOCOL, TELEMETRY_STREAM_ID_HIGH_RATE_SNAPSHOT):
        raise ValueError("stream_id 仅支持 0xF1(LegacyProtocol) 或 0x00(HighRateSnapshot)")
    return stream


def _parse_hand_calib_point(value: str) -> int:
    point_text = _strip_placeholder(value).lower()
    mapping = {
        "1": HAND_CALIB_POINT_JX_CENTER,
        "jx_center": HAND_CALIB_POINT_JX_CENTER,
        "2": HAND_CALIB_POINT_JX_MAX,
        "jx_max": HAND_CALIB_POINT_JX_MAX,
        "3": HAND_CALIB_POINT_JX_MIN,
        "jx_min": HAND_CALIB_POINT_JX_MIN,
        "4": HAND_CALIB_POINT_JY_CENTER,
        "jy_center": HAND_CALIB_POINT_JY_CENTER,
        "5": HAND_CALIB_POINT_JY_MAX,
        "jy_max": HAND_CALIB_POINT_JY_MAX,
        "6": HAND_CALIB_POINT_JY_MIN,
        "jy_min": HAND_CALIB_POINT_JY_MIN,
        "7": HAND_CALIB_POINT_TRIG_START,
        "trig_start": HAND_CALIB_POINT_TRIG_START,
        "8": HAND_CALIB_POINT_TRIG_MAX,
        "trig_max": HAND_CALIB_POINT_TRIG_MAX,
    }
    if point_text not in mapping:
        raise ValueError("point 仅支持 1..8 或 jx_center/jx_max/jx_min/jy_center/jy_max/jy_min/trig_start/trig_max")
    return mapping[point_text]


def _resolve_hand_index(client: QnbotClient, idx_text: Optional[str] = None) -> int:
    if idx_text is not None:
        return _parse_int(idx_text)
    info_list = client.get_hand_info_list()
    ready = [entry.handset_index for entry in info_list.entries]
    if len(ready) == 1:
        return ready[0]
    if not ready:
        raise ValueError("当前未检测到在线手柄，请先连接手柄，或显式指定 idx")
    raise ValueError(f"检测到多个手柄在线 {ready}，请显式指定 idx")


def _resolve_imu_index(client: QnbotClient, idx_text: Optional[str] = None) -> int:
    if idx_text is not None:
        return _parse_int(idx_text)
    info_list = client.get_imu_info_list()
    ready = [entry.imu_index for entry in info_list.entries]
    if len(ready) == 1:
        return ready[0]
    if not ready:
        # 某些固件在特定状态下不会列出 IMU，但索引 0 仍可用于 Mag 命令。
        return 0
    raise ValueError(f"检测到多个 IMU 可用 {ready}，请显式指定 idx")


def _get_hand_axes_from_snap(snap: Optional[TelemetrySnapshot], handset_index: int) -> Optional[List[int]]:
    if snap is None:
        return None
    if handset_index == 0:
        return list(snap.joystick_left)
    if handset_index == 1:
        return list(snap.joystick_right)
    return None


def _sample_hand_mean(
    handset_index: int,
    telemetry_lock: threading.Lock,
    telemetry_state: Dict[str, object],
    prompt: str,
    set_guide: Callable[[str, str], None],
    duration_s: float = 1.0,
) -> Optional[List[int]]:
    set_guide(prompt, "回车开始采样，q取消；采样中请保持当前姿态稳定。")
    while True:
        ch = _read_key_nonblock()
        if ch is None:
            time.sleep(0.02)
            continue
        if ch in ("\n", "\r"):
            break
        if ch.lower() == "q":
            return None
    set_guide(prompt, f"正在采样 {duration_s:.1f}s ...")
    deadline = time.time() + duration_s
    samples: List[List[int]] = []
    while time.time() < deadline:
        with telemetry_lock:
            snap = telemetry_state.get("latest_snap")
        values = _get_hand_axes_from_snap(snap, handset_index) if isinstance(snap, TelemetrySnapshot) else None
        if values is not None and len(values) >= 4:
            samples.append(values[:4])
        time.sleep(0.01)
    if not samples:
        set_guide(prompt, "采样失败：未获取到手柄数据，请检查遥测链路。")
        return None
    means = [
        int(sum(item[i] for item in samples) / len(samples))
        for i in range(4)
    ]
    set_guide(
        prompt,
        f"采样结果 X={means[0]},Y={means[1]} keys=0x{means[2] & 0xFFFF:04X} trigger={means[3]}",
    )
    return means


def _run_hand_calib_wizard(
    client: QnbotClient,
    handset_index: int,
    telemetry_lock: threading.Lock,
    telemetry_state: Dict[str, object],
    input_state: Dict[str, str],
    telemetry_print: List[bool],
    telemetry_style: List[str],
    joystick_extended_keys: List[bool],
) -> None:
    old_print = telemetry_print[0]
    old_style = telemetry_style[0]
    telemetry_print[0] = False
    print(f"开始手柄引导校准: idx={handset_index}")

    before = client.get_hand_calib_params(handset_index)
    _print_hand_calib_params(before)

    stop_stream = threading.Event()
    ui_lock = threading.Lock()
    ui_state = {
        "guide1": "准备中...",
        "guide2": "正在等待遥测数据。",
        "left": "左手柄: -",
        "right": "右手柄: -",
    }

    def _set_guide(line1: str, line2: str) -> None:
        with ui_lock:
            ui_state["guide1"] = line1
            ui_state["guide2"] = line2

    def _hand_stream_loop() -> None:
        first_draw = True
        while not stop_stream.is_set():
            with telemetry_lock:
                snap = telemetry_state.get("latest_snap")
            if isinstance(snap, TelemetrySnapshot):
                left = f"左手柄: {_format_joystick('', snap.joystick_left, extended_keys=joystick_extended_keys[0])}"
                right = f"右手柄: {_format_joystick('', snap.joystick_right, extended_keys=joystick_extended_keys[0])}"
                with ui_lock:
                    ui_state["left"] = left
                    ui_state["right"] = right

            with ui_lock:
                guide1 = ui_state["guide1"]
                guide2 = ui_state["guide2"]
                left_line = ui_state["left"]
                right_line = ui_state["right"]

            # 固定 4 行刷新：2 行引导 + 2 行手柄状态。
            # 引导信息与手柄数据都在固定区域内更新，避免滚屏刷掉提示内容。
            if first_draw:
                _lwrite(f"{guide1}\n")
                _lwrite(f"{guide2}\n")
                _lwrite(f"{left_line}\n")
                _lwrite(f"{right_line}\n")
                first_draw = False
            else:
                _lwrite("\x1b[4F")
                _lwrite("\x1b[2K")
                _lwrite(f"{guide1}\n")
                _lwrite("\x1b[2K")
                _lwrite(f"{guide2}\n")
                _lwrite("\x1b[2K")
                _lwrite(f"{left_line}\n")
                _lwrite("\x1b[2K")
                _lwrite(f"{right_line}\n")
            sys.stdout.flush()
            time.sleep(0.10)

    stream_thread = threading.Thread(target=_hand_stream_loop, daemon=True)
    stream_thread.start()

    restore_done = False

    def _restore_calib_output() -> None:
        nonlocal restore_done
        if restore_done:
            return
        try:
            client.hand_set_calib_output(handset_index, before.calib_output_enabled)
        except Exception:
            pass
        restore_done = True

    try:
        out_res = client.hand_set_calib_output(handset_index, 0)
        if out_res.status_code != 0x00:
            raise RuntimeError(f"关闭校准输出失败: status=0x{out_res.status_code:02X}")
        start_res = client.hand_calib_start(handset_index)
        if start_res.status_code != 0x00:
            raise RuntimeError(f"校准启动失败: status=0x{start_res.status_code:02X}")

        def _sample_enter_mean(step_title: str, duration_s: float = 1.0) -> List[int]:
            data = _sample_hand_mean(
                handset_index,
                telemetry_lock,
                telemetry_state,
                step_title,
                _set_guide,
                duration_s=duration_s,
            )
            if data is None:
                raise RuntimeError("采样失败：未获取有效手柄数据")
            return data

        def _sample_enter_extrema(step_title: str, duration_s: float = 6.0) -> List[int]:
            _set_guide(step_title, "回车开始覆盖采样，q取消。")
            while True:
                ch = _read_key_nonblock()
                if ch is None:
                    time.sleep(0.02)
                    continue
                if ch in ("\n", "\r"):
                    break
                if ch.lower() == "q":
                    raise RuntimeError("校准已取消")
            deadline = time.time() + duration_s
            min_x, max_x = 0xFFFF, 0
            min_y, max_y = 0xFFFF, 0
            got = False
            while time.time() < deadline:
                with telemetry_lock:
                    snap = telemetry_state.get("latest_snap")
                values = _get_hand_axes_from_snap(snap, handset_index) if isinstance(snap, TelemetrySnapshot) else None
                if values is not None and len(values) >= 2:
                    x, y = int(values[0]), int(values[1])
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
                    got = True
                remain = max(0.0, deadline - time.time())
                _set_guide(
                    step_title,
                    f"采样中 剩余{remain:.1f}s X=({min_x},{max_x}) Y=({min_y},{max_y})",
                )
                time.sleep(0.01)
            if not got or min_x > max_x or min_y > max_y:
                raise RuntimeError("覆盖采样失败：未得到有效极值")
            _set_guide(step_title, f"采样结果 X=({min_x},{max_x}) Y=({min_y},{max_y})")
            return [min_x, max_x, min_y, max_y]

        def _sample_enter_trigger_max(step_title: str, duration_s: float = 3.0) -> int:
            _set_guide(step_title, "回车开始采样，q取消。")
            while True:
                ch = _read_key_nonblock()
                if ch is None:
                    time.sleep(0.02)
                    continue
                if ch in ("\n", "\r"):
                    break
                if ch.lower() == "q":
                    raise RuntimeError("校准已取消")
            deadline = time.time() + duration_s
            trig_max = -1
            got = False
            while time.time() < deadline:
                with telemetry_lock:
                    snap = telemetry_state.get("latest_snap")
                values = _get_hand_axes_from_snap(snap, handset_index) if isinstance(snap, TelemetrySnapshot) else None
                if values is not None and len(values) >= 4:
                    trig = int(values[3])
                    trig_max = max(trig_max, trig)
                    got = True
                remain = max(0.0, deadline - time.time())
                _set_guide(step_title, f"采样中 剩余{remain:.1f}s trig_max={max(trig_max, 0)}")
                time.sleep(0.01)
            if not got:
                raise RuntimeError("扳机采样失败：未获取有效数据")
            _set_guide(step_title, f"采样结果 trig_max={trig_max}")
            return trig_max

        # SMART_CALIB_STEPS 固定 8 步
        # 1) 左滑后松开 -> 左侧静止点（均值）
        left = _sample_enter_mean("步骤1/8：左滑摇杆后松开，采集左侧静止点。", duration_s=1.0)
        # 2) 右滑后松开 -> 右侧静止点（均值）
        right = _sample_enter_mean("步骤2/8：右滑摇杆后松开，采集右侧静止点。", duration_s=1.0)
        # 3) 上滑后松开 -> 上侧静止点（均值）
        up = _sample_enter_mean("步骤3/8：上滑摇杆后松开，采集上侧静止点。", duration_s=1.0)
        # 4) 下滑后松开 -> 下侧静止点（均值）
        down = _sample_enter_mean("步骤4/8：下滑摇杆后松开，采集下侧静止点。", duration_s=1.0)
        # 5) 绕圈采集极值（6s）
        extrema = _sample_enter_extrema("步骤5/8：绕圈旋转摇杆约6秒，自动记录X/Y极值。", duration_s=6.0)
        # 6) 松开扳机 -> 起始值（均值）
        trig_start_data = _sample_enter_mean("步骤6/8：松开扳机，采集扳机起始值。", duration_s=1.0)
        # 7) 扳机到底 -> 最大值（3s极大值）
        trig_max = _sample_enter_trigger_max("步骤7/8：按下扳机到底约3秒，自动记录扳机最大值。", duration_s=3.0)

        x_center = int(round((left[0] + right[0]) / 2.0))
        y_center = int(round((up[1] + down[1]) / 2.0))
        x_min, x_max, y_min, y_max = extrema[0], extrema[1], extrema[2], extrema[3]
        trig_start = int(trig_start_data[3])

        def _u16(v: int) -> int:
            return max(0, min(0xFFFF, int(v)))

        values = {
            HAND_CALIB_POINT_JX_CENTER: _u16(x_center),
            HAND_CALIB_POINT_JX_MAX: _u16(x_max),
            HAND_CALIB_POINT_JX_MIN: _u16(x_min),
            HAND_CALIB_POINT_JY_CENTER: _u16(y_center),
            HAND_CALIB_POINT_JY_MAX: _u16(y_max),
            HAND_CALIB_POINT_JY_MIN: _u16(y_min),
            HAND_CALIB_POINT_TRIG_START: _u16(trig_start),
            HAND_CALIB_POINT_TRIG_MAX: _u16(trig_max),
        }

        # 8) 写入并结束
        _set_guide("步骤8/8：写入校准参数并结束校准。", "回车确认写入，q取消。")
        while True:
            ch = _read_key_nonblock()
            if ch is None:
                time.sleep(0.02)
                continue
            if ch in ("\n", "\r"):
                break
            if ch.lower() == "q":
                raise RuntimeError("校准已取消")

        _set_guide("步骤8/8：正在写入参数...", "请稍候。")
        for point, value in values.items():
            set_res = client.hand_set_calib_param(handset_index, point, value)
            if set_res.status_code != 0x00:
                raise RuntimeError(
                    f"写参失败: point={point}({_hand_calib_point_desc(point)}) "
                    f"value={value} status=0x{set_res.status_code:02X}"
                )
            # 与 handler_tool 保持一致，给手柄侧参数落地留出处理窗口，降低 finish 的 NOT_READY 概率。
            time.sleep(0.05)

        fin_res = None
        last_status = 0xFF
        for attempt in range(1, 6):
            fin_res = client.hand_calib_finish(handset_index)
            last_status = fin_res.status_code
            if last_status == 0x00:
                break
            _set_guide("步骤8/8：结束校准重试中...", f"attempt={attempt}/5 status=0x{last_status:02X}")
            time.sleep(0.20)

        if fin_res is None or fin_res.status_code != 0x00:
            # 部分固件在写参后短时会拒绝 finish；由于 finish 语义本质是恢复输出，
            # 这里做一次兼容回退，尽可能恢复到可用状态。
            fallback_res = client.hand_set_calib_output(handset_index, 1)
            if fallback_res.status_code != 0x00:
                raise RuntimeError(
                    f"校准结束失败: finish=0x{last_status:02X}, fallback_set_output=0x{fallback_res.status_code:02X}"
                )
        _restore_calib_output()

        after = client.get_hand_calib_params(handset_index)
        _set_guide("校准完成。", "回读参数如下。")
        time.sleep(0.20)
        stop_stream.set()
        stream_thread.join(timeout=1.0)
        print("校准完成，写入参数如下：")
        for point, value in values.items():
            print(f"  point={point}({_hand_calib_point_desc(point)}) value={value}")
        print("回读参数如下：")
        _print_hand_calib_params(after)
    finally:
        _restore_calib_output()
        stop_stream.set()
        stream_thread.join(timeout=1.0)
        telemetry_print[0] = old_print
        telemetry_style[0] = old_style


def _run_imu_mag_wizard(
    client: QnbotClient,
    imu_index: int,
    input_state: Dict[str, str],
) -> None:
    print(f"开始 IMU 磁力计引导校准: idx={imu_index}")
    start = client.imu_mag_calibrate(imu_index, 0x01)
    print(
        f"IMU.Mag.Start: status=0x{start.status_code:02X}({_status_text(start.status_code)}) "
        f"idx={start.imu_index} state={start.state} err=0x{start.error_code:04X}"
    )
    if start.status_code != 0x00:
        return
    print("操作提示：请缓慢分别绕 X / Y / Z 三个轴旋转设备，覆盖尽量多方向。")
    print("完成后按回车结束并保存（建议旋转至少 10~20 秒）。")
    line = _read_command(threading.Event(), input_state)
    if line is None:
        print("检测到中断，正在尝试结束校准并保存。")
    finish = client.imu_mag_calibrate(imu_index, 0x02)
    print(
        f"IMU.Mag.Finish: status=0x{finish.status_code:02X}({_status_text(finish.status_code)}) "
        f"idx={finish.imu_index} state={finish.state} err=0x{finish.error_code:04X}"
    )


def _run_haptics_wizard(
    client: QnbotClient,
    input_state: Dict[str, str],
    telemetry_lock: threading.Lock,
    telemetry_state: Dict[str, object],
    runtime_support: Optional[Dict[str, bool]] = None,
) -> None:
    if runtime_support is not None and not runtime_support.get("haptics", True):
        print(_runtime_unsupported_message("haptics.*"))
        return

    def _prompt_int(label: str, default: int) -> Optional[int]:
        while True:
            raw = input(f"{_tr(label)} ({_tr('默认')} {default}): ").strip()
            if not raw:
                return default
            if raw.lower() == "q":
                return None
            try:
                return _parse_int(raw)
            except Exception as e:
                print(f"{_tr('输入无效')}：{e}")

    def _step_gate(title: str, detail: str) -> bool:
        print(_tr(title))
        print(_tr(detail))
        return _confirm_or_abort(_tr("按 Enter 继续，按 q 退出。"), input_state)

    print(_tr("开始 Haptics 引导调试。"))
    print(_tr("提示：按 q 可随时中断。"))

    probe_results: List[Tuple[int, int]] = []
    usable_channels: List[int] = []
    handset_probe_results: List[Tuple[int, int]] = []
    usable_handsets: List[int] = []
    probe_candidates = list(range(0, 8))
    print(_tr("步骤0/6：验证默认通道并补充扫描"))
    for ch in probe_candidates:
        try:
            res = client.haptics_set_output(ch, 1, 0, 10)
            probe_results.append((ch, res.status_code))
            if res.status_code == 0x00:
                usable_channels.append(ch)
        except Exception:
            probe_results.append((ch, 0xFF))

    for idx in range(0, 2):
        try:
            res = client.haptics_drv_get_cal_status(idx)
            handset_probe_results.append((idx, res.status_code))
            if res.status_code == 0x00:
                usable_handsets.append(idx)
        except Exception:
            handset_probe_results.append((idx, 0xFF))

    preferred_channel = usable_channels[0] if usable_channels else 0
    preferred_handset = usable_handsets[0] if usable_handsets else 0
    print(_tr("通道探测结果："))
    for ch, st in probe_results:
        if st == 0x03:
            continue
        print(f"  channel={ch} status=0x{st:02X}({_status_text(st)})")
    print(_tr("手柄探测结果："))
    for idx, st in handset_probe_results:
        if st == 0x03:
            continue
        print(f"  handset={idx} status=0x{st:02X}({_status_text(st)})")
    if usable_channels:
        candidates_text = ", ".join(str(i) for i in usable_channels)
    else:
        candidates_text = _tr("未探测到返回 OK 的通道")
    if usable_handsets:
        handsets_text = ", ".join(str(i) for i in usable_handsets)
    else:
        handsets_text = _tr("未探测到返回 OK 的手柄")
    detail = (
        f"{_tr('推荐手柄')}: {preferred_handset}, "
        f"{_tr('推荐通道')}: {preferred_channel}, "
        f"{_tr('可用候选手柄')}: {handsets_text}, "
        f"{_tr('可用候选通道')}: {candidates_text}\n"
        f"{_tr('说明')}: {_tr('自动探测仅作参考；若现场已知通道，请在下一步手动输入覆盖。')}"
    )
    if not _step_gate("步骤1/6：确认探测结果", detail):
        print(_tr("Haptics 引导调试已取消。"))
        return

    handset_index = _prompt_int("手柄索引 handset_index", preferred_handset)
    if handset_index is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    channel_id = _prompt_int("输出通道 channel_id", preferred_channel)
    if channel_id is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    amplitude = _prompt_int("试播幅度 amplitude (0~100)", 100)
    if amplitude is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    pattern_id = _prompt_int("试播模式 pattern_id", 0)
    if pattern_id is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    duration_ms = _prompt_int("试播时长 duration_ms", 500)
    if duration_ms is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    effect_id = _prompt_int("预置效果 effect_id", 47)
    if effect_id is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    realtime_amp = _prompt_int("实时振幅 realtime_amp", 96)
    if realtime_amp is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    enable = _prompt_int("使能开关 enable (0/1)", 1)
    if enable is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    mode = _prompt_int("模式 mode", 0)
    if mode is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    fixed_strength = _prompt_int("固定反馈强度 fixed_strength (0~100)", 80)
    if fixed_strength is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    timeout_ms = _prompt_int("超时值 timeout_ms", 2000)
    if timeout_ms is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    trigger_threshold = _prompt_int("扳机阈值 threshold (0~3584)", 500)
    if trigger_threshold is None:
        print(_tr("Haptics 引导调试已取消。"))
        return
    trigger_poll_ms = _prompt_int("扳机轮询周期 poll_ms", 100)
    if trigger_poll_ms is None:
        print(_tr("Haptics 引导调试已取消。"))
        return

    summary: List[str] = []

    if not _step_gate("步骤2/6：查询驱动校准状态", f"handset_index={handset_index}"):
        print(_tr("Haptics 引导调试已取消。"))
        return
    try:
        cal = client.haptics_drv_get_cal_status(handset_index)
        print(
            f"Haptics.DrvGetCalStatus: status=0x{cal.status_code:02X}({_status_text(cal.status_code)}) "
            f"idx={cal.handset_index} calibrated={cal.calibrated}"
        )
        summary.append(f"DrvGetCalStatus=0x{cal.status_code:02X} calibrated={cal.calibrated}")
    except Exception as e:
        print(f"{_tr('查询失败')}: {e}")
        return

    if not _step_gate(
        "步骤3/6：试播 Haptics 输出",
        f"channel_id={channel_id} amplitude={amplitude} pattern_id={pattern_id} duration_ms={duration_ms} "
        f"effect_id={effect_id}\n"
        f"{_tr('说明')}: {_tr('按回车后会立即先下发 SetOutput，再补发一次 VibratePlay 预置效果，用于增强现场体感确认。')}",
    ):
        print(_tr("Haptics 引导调试已取消。"))
        return
    try:
        print(_tr("即将触发一次联合试播，请留意目标手柄震动。"))
        print(_tr("正在立即发送 Haptics.SetOutput..."))
        out = client.haptics_set_output(channel_id, amplitude, pattern_id, duration_ms)
        print(
            f"Haptics.SetOutput: status=0x{out.status_code:02X}({_status_text(out.status_code)}) "
            f"channel={out.channel_id}"
        )
        summary.append(f"SetOutput=0x{out.status_code:02X} channel={out.channel_id}")
        print(_tr("正在立即发送 Haptics.VibratePlay 预置效果..."))
        play_probe = client.haptics_vibrate_play(handset_index, effect_id)
        print(
            f"Haptics.VibratePlay: status=0x{play_probe.status_code:02X}({_status_text(play_probe.status_code)}) "
            f"idx={play_probe.handset_index} effect_id={play_probe.effect_id}"
        )
        summary.append(f"TrialVibratePlay=0x{play_probe.status_code:02X} effect_id={play_probe.effect_id}")
        print(_tr("若设备支持该手柄/通道，此刻就应该能感知到试播震动。"))
        time.sleep(min(max(0.5, duration_ms / 1000.0 + 0.1), 2.0))
    except Exception as e:
        print(f"{_tr('试播失败')}: {e}")
        return

    if not _step_gate("步骤4/6：停止输出", f"handset_index={handset_index}"):
        print(_tr("Haptics 引导调试已取消。"))
        return
    try:
        stop = client.haptics_vibrate_stop(handset_index)
        print(
            f"Haptics.VibrateStop: status=0x{stop.status_code:02X}({_status_text(stop.status_code)}) "
            f"idx={stop.handset_index}"
        )
        summary.append(f"VibrateStop=0x{stop.status_code:02X}")
    except Exception as e:
        print(f"{_tr('停止失败')}: {e}")
        return

    if not _step_gate("步骤5/6：读取强度", f"handset_index={handset_index}"):
        print(_tr("Haptics 引导调试已取消。"))
        return
    try:
        intensity = client.haptics_get_intensity(handset_index)
        print(
            f"Haptics.GetIntensity: status=0x{intensity.status_code:02X}({_status_text(intensity.status_code)}) "
            f"idx={intensity.handset_index} intensity={intensity.intensity}"
        )
        summary.append(f"GetIntensity=0x{intensity.status_code:02X} value={intensity.intensity}")
    except Exception as e:
        print(f"{_tr('读取强度失败')}: {e}")
        summary.append("GetIntensity=TIMEOUT_OR_UNSUPPORTED")

    if not _step_gate(
        "步骤6/7：固定反馈强度测试",
        f"enable={enable} mode={mode} fixed_strength={fixed_strength} timeout_ms={timeout_ms}\n"
        f"{_tr('说明')}: {_tr('按回车后会按顺序立即执行 enable/mode/timeout，并把固定强度 0~100 映射为内部压力值后持续下发约 3 秒。')}",
    ):
        print(_tr("Haptics 引导调试已取消。"))
        return
    try:
        print(_tr("正在立即执行调参命令链路..."))
        en_res = client.haptics_set_enable(handset_index, enable)
        print(
            f"Haptics.SetEnable: status=0x{en_res.status_code:02X}({_status_text(en_res.status_code)}) "
            f"idx={en_res.handset_index} enable={en_res.enable}"
        )
        summary.append(f"SetEnable=0x{en_res.status_code:02X} value={en_res.enable}")
        mode_res = client.haptics_set_mode(handset_index, mode)
        print(
            f"Haptics.SetMode: status=0x{mode_res.status_code:02X}({_status_text(mode_res.status_code)}) "
            f"idx={mode_res.handset_index} mode={mode_res.mode}"
        )
        summary.append(f"SetMode=0x{mode_res.status_code:02X} value={mode_res.mode}")
        timeout_res = client.haptics_set_timeout(handset_index, timeout_ms)
        print(
            f"Haptics.SetTimeout: status=0x{timeout_res.status_code:02X}({_status_text(timeout_res.status_code)}) "
            f"idx={timeout_res.handset_index} timeout_ms={timeout_res.timeout_ms}"
        )
        summary.append(f"SetTimeout=0x{timeout_res.status_code:02X} value={timeout_res.timeout_ms}")
        fixed_pressure = _clamp(int(round((fixed_strength / 100.0) * 4095)), 0, 4095)
        print(
            f"{_tr('固定反馈强度')}={fixed_strength}/100 "
            f"{_tr('映射压力值')}={fixed_pressure}/4095"
        )
        print(_tr("开始固定反馈强度测试（约3秒），请留意当前强度是否容易感知。"))
        fixed_loop_ok = True
        fixed_loop_timeout_count = 0
        fixed_hold_deadline = time.time() + 3.0
        while time.time() < fixed_hold_deadline:
            try:
                pressure_loop = client.haptics_set_pressure(handset_index, fixed_pressure)
                if pressure_loop.status_code != 0x00:
                    fixed_loop_ok = False
            except Exception:
                fixed_loop_ok = False
                fixed_loop_timeout_count += 1
            time.sleep(0.1)
        if fixed_loop_timeout_count:
            print(f"{_tr('固定反馈测试过程中出现超时次数')}: {fixed_loop_timeout_count}")
        summary.append(
            f"FixedFeedback={'OK' if fixed_loop_ok else 'PARTIAL'} strength={fixed_strength} "
            f"pressure={fixed_pressure} timeouts={fixed_loop_timeout_count}"
        )
    except Exception as e:
        print(f"{_tr('调参失败')}: {e}")
        return

    if not _step_gate(
        "步骤7/7：进入扳机联动模拟",
        f"handset_index={handset_index} threshold={trigger_threshold} poll_ms={trigger_poll_ms}\n"
        f"{_tr('说明')}: {_tr('按回车后进入实时联动模式：扳机值超过阈值后开始反馈；按 q 可随时退出该模式。')}",
    ):
        print(_tr("Haptics 引导调试已取消。"))
        return
    try:
        _run_haptics_trigger_sim(
            client,
            handset_index,
            trigger_threshold,
            trigger_poll_ms,
            telemetry_lock,
            telemetry_state,
        )
        summary.append(
            f"TriggerSim=entered threshold={trigger_threshold} poll_ms={trigger_poll_ms}"
        )
    except Exception as e:
        print(f"{_tr('扳机联动模拟失败')}: {e}")
        summary.append("TriggerSim=FAILED")

    print(_tr("Haptics 引导调试完成。"))
    print(_tr("现场摘要："))
    print(f"  handset_index={handset_index} channel_id={channel_id}")
    print(f"  amplitude={amplitude} pattern_id={pattern_id} duration_ms={duration_ms}")
    print(f"  effect_id={effect_id} realtime_amp={realtime_amp}")
    print(
        f"  enable={enable} mode={mode} fixed_strength={fixed_strength} "
        f"timeout_ms={timeout_ms} threshold={trigger_threshold} poll_ms={trigger_poll_ms}"
    )
    for item in summary:
        print(f"  - {item}")


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def _get_trigger_from_snapshot(snapshot: Optional[TelemetrySnapshot], handset_index: int) -> Optional[int]:
    if snapshot is None:
        return None
    values = snapshot.joystick_left if handset_index == 0 else snapshot.joystick_right
    if not values or len(values) < 4:
        return None
    return int(values[3])


def _run_haptics_force_test(
    client: QnbotClient,
    handset_index: int,
    pressure: int,
    mode: int,
    timeout_ms: int,
    hold_s: float,
) -> None:
    pressure = _clamp(int(pressure), 0, 4095)
    mode = _clamp(int(mode), 0, 255)
    timeout_ms = _clamp(int(timeout_ms), 100, 60000)

    print(f"{_tr('开始力反馈定压测试')}: idx={handset_index} pressure={pressure} mode={mode} timeout_ms={timeout_ms} hold_s={hold_s:.2f}")
    en_res = client.haptics_set_enable(handset_index, 1)
    print(
        f"Haptics.SetEnable: status=0x{en_res.status_code:02X}({_status_text(en_res.status_code)}) "
        f"idx={en_res.handset_index} enable={en_res.enable}"
    )
    if en_res.status_code != 0x00:
        return

    mode_res = client.haptics_set_mode(handset_index, mode)
    print(
        f"Haptics.SetMode: status=0x{mode_res.status_code:02X}({_status_text(mode_res.status_code)}) "
        f"idx={mode_res.handset_index} mode={mode_res.mode}"
    )
    if mode_res.status_code != 0x00:
        client.haptics_set_enable(handset_index, 0)
        return

    timeout_res = client.haptics_set_timeout(handset_index, timeout_ms)
    print(
        f"Haptics.SetTimeout: status=0x{timeout_res.status_code:02X}({_status_text(timeout_res.status_code)}) "
        f"idx={timeout_res.handset_index} timeout_ms={timeout_res.timeout_ms}"
    )
    if timeout_res.status_code != 0x00:
        client.haptics_set_enable(handset_index, 0)
        return

    print(_tr("按 q 停止测试。"))
    started = time.time()
    last_line = ""
    try:
        while True:
            ch = _read_key_nonblock()
            if ch and ch.lower() == "q":
                break
            if hold_s > 0 and (time.time() - started) >= hold_s:
                break
            res = client.haptics_set_pressure(handset_index, pressure)
            elapsed = time.time() - started
            line = (
                f"Haptics.SetPressure: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                f"idx={res.handset_index} pressure={res.pressure} elapsed={elapsed:.1f}s"
            )
            if last_line:
                _lwrite("\r")
            _lwrite(line)
            _lwrite(" " * max(0, len(last_line) - len(line)))
            sys.stdout.flush()
            last_line = line
            time.sleep(0.10)
    finally:
        if last_line:
            _lwrite("\n")
        try:
            client.haptics_set_pressure(handset_index, 0)
        except Exception:
            pass
        try:
            client.haptics_set_enable(handset_index, 0)
        except Exception:
            pass
        print(_tr("力反馈测试结束，已关闭输出。"))


def _run_haptics_trigger_sim(
    client: QnbotClient,
    handset_index: int,
    threshold: int,
    poll_ms: int,
    telemetry_lock: threading.Lock,
    telemetry_state: Dict[str, object],
) -> None:
    threshold = _clamp(int(threshold), 0, 3584)
    poll_ms = _clamp(int(poll_ms), 20, 500)
    trigger_min = 512
    trigger_max = 3584
    effective_threshold = max(trigger_min, threshold)
    interval_s = poll_ms / 1000.0

    print(
        f"{_tr('开始扳机震感模拟')}: idx={handset_index} threshold={threshold} "
        f"effective_start={effective_threshold} poll_ms={poll_ms}"
    )
    print(_tr("说明：扳机值按 512~3584 映射为压力值；当 threshold 小于 512 时，仍以 512 作为实际起点。"))
    en_res = client.haptics_set_enable(handset_index, 1)
    print(
        f"Haptics.SetEnable: status=0x{en_res.status_code:02X}({_status_text(en_res.status_code)}) "
        f"idx={en_res.handset_index} enable={en_res.enable}"
    )
    if en_res.status_code != 0x00:
        return
    timeout_res = client.haptics_set_timeout(handset_index, 2000)
    print(
        f"Haptics.SetTimeout: status=0x{timeout_res.status_code:02X}({_status_text(timeout_res.status_code)}) "
        f"idx={timeout_res.handset_index} timeout_ms={timeout_res.timeout_ms}"
    )
    print(_tr("按 q 停止测试。"))

    last_pressure = -1
    last_line = ""
    try:
        while True:
            ch = _read_key_nonblock()
            if ch and ch.lower() == "q":
                break

            with telemetry_lock:
                snap = telemetry_state.get("latest_snap")

            trigger_val = _get_trigger_from_snapshot(snap, handset_index)
            if trigger_val is None:
                line = _tr("等待遥测数据...")
                if last_line:
                    _lwrite("\r")
                _lwrite(line)
                _lwrite(" " * max(0, len(last_line) - len(line)))
                sys.stdout.flush()
                last_line = line
                time.sleep(interval_s)
                continue

            trigger_val = _clamp(trigger_val, 0, trigger_max)
            if trigger_val <= effective_threshold:
                pressure = 0
            else:
                effective_range = max(1, trigger_max - effective_threshold)
                pressure = int(((trigger_val - effective_threshold) / float(effective_range)) * 4095)
                pressure = _clamp(pressure, 0, 4095)

            status_code = 0x00
            if pressure != last_pressure:
                res = client.haptics_set_pressure(handset_index, pressure)
                status_code = res.status_code
                last_pressure = pressure

            line = (
                f"{_tr('当前扳机值')}={trigger_val} "
                f"{_tr('扳机阈值')}={threshold} "
                f"{_tr('实际起点')}={effective_threshold} "
                f"{_tr('当前压力值')}={pressure}/4095 "
                f"status=0x{status_code:02X}"
            )
            if last_line:
                _lwrite("\r")
            _lwrite(line)
            _lwrite(" " * max(0, len(last_line) - len(line)))
            sys.stdout.flush()
            last_line = line
            time.sleep(interval_s)
    finally:
        if last_line:
            _lwrite("\n")
        try:
            client.haptics_set_pressure(handset_index, 0)
        except Exception:
            pass
        try:
            client.haptics_set_enable(handset_index, 0)
        except Exception:
            pass
        print(_tr("扳机震感模拟结束，已关闭输出。"))


def _run_wl_pair_wizard(
    client: QnbotClient,
    role_mode: Optional[int],
    timeout_s: float,
) -> None:
    if timeout_s <= 0:
        raise ValueError("timeout_s 必须大于 0")
    pre = client.get_wireless_status_info()
    print("配对前状态：")
    _print_wireless_status(pre)

    if pre.pairing_busy:
        print("检测到已有配对流程在运行，先尝试取消旧流程...")
        client.wireless_pair_cancel()
        time.sleep(0.6)
    pre2 = client.get_wireless_status_info()
    if pre2.in_config_mode:
        print("检测到当前处于配置态，先退出配置态...")
        client.wireless_exit_config()
        time.sleep(0.6)
        pre2 = client.get_wireless_status_info()
    print("预处理后状态：")
    _print_wireless_status(pre2)

    start = client.wireless_pair_start(role_mode=role_mode)
    _print_wireless_command_result("Wireless.PairStart", start)
    if start.status_code != 0x00:
        return

    print("开始轮询配对状态（按 q 可中止并发送取消）。")
    first_draw = True
    last_key = None
    last_progress_text = ""
    last_render_time = 0.0
    render_interval_s = 0.20
    started_at = time.time()
    last_line = "Wireless 状态: -"
    last_err = ""
    poll_timeout_s = 0.25
    step6_deadline: Optional[float] = None
    progress_text = "准备中"

    def _render(status_line: str, progress_info: str, err_text: str = "") -> None:
        nonlocal first_draw
        guide = "轮询中（按 q 取消）"
        if first_draw:
            _lwrite(f"{guide}\n")
            _lwrite(f"{status_line}\n")
            _lwrite(f"状态: {progress_info} {err_text}\n")
            first_draw = False
        else:
            _lwrite("\x1b[3F")
            _lwrite("\x1b[2K")
            _lwrite(f"{guide}\n")
            _lwrite("\x1b[2K")
            _lwrite(f"{status_line}\n")
            _lwrite("\x1b[2K")
            _lwrite(f"状态: {progress_info} {err_text}\n")
        sys.stdout.flush()

    while True:
        ch = _read_key_nonblock()
        if ch and ch.lower() == "q":
            # 先把行刷新区域收尾，避免后续输出覆盖。
            if not first_draw:
                _lwrite("\n")
                sys.stdout.flush()
            print("收到中止指令，正在取消配对流程...")
            try:
                client.wireless_pair_cancel()
            except Exception:
                pass
            time.sleep(0.6)
            final_st = client.get_wireless_status_info()
            _print_wireless_status(final_st)
            print(f"配对流程已进入终态：{_pair_result_text(final_st.pair_result)}")
            return
        try:
            st = client.get_wireless_status_info(timeout=poll_timeout_s)
            last_err = ""
            key = (
                st.pairing_busy,
                st.pair_step,
                st.pair_result,
                st.in_config_mode,
                st.link_state,
                st.paired_device_name,
            )
            key_changed = (key != last_key)
            if key != last_key:
                last_line = _wireless_status_line(st)
                last_key = key

            if st.pair_step == 6 and step6_deadline is None:
                step6_deadline = time.time() + timeout_s
            progress_text = "正在等待配对结果" if step6_deadline is not None else "准备中"

            now = time.time()
            status_changed = key_changed or (progress_text != last_progress_text)
            should_render = first_draw or status_changed or (now - last_render_time >= render_interval_s)
            if should_render:
                _render(last_line, progress_text, last_err)
                last_render_time = now
                last_progress_text = progress_text

            if (
                st.pairing_busy == 0 and
                st.pair_result in (
                    WIRELESS_PAIR_RESULT_OK,
                    WIRELESS_PAIR_RESULT_TIMEOUT,
                    WIRELESS_PAIR_RESULT_CANCELED,
                    WIRELESS_PAIR_RESULT_ERROR,
                )
            ):
                if not first_draw:
                    _lwrite("\n")
                    sys.stdout.flush()
                print(f"配对流程已进入终态：{_pair_result_text(st.pair_result)}")
                return

            if step6_deadline is not None and time.time() >= step6_deadline:
                if not first_draw:
                    _lwrite("\n")
                    sys.stdout.flush()
                try:
                    client.wireless_pair_cancel()
                except Exception:
                    pass
                time.sleep(0.6)
                final_st = client.get_wireless_status_info()
                _print_wireless_status(final_st)
                print(f"配对流程已进入终态：{_pair_result_text(final_st.pair_result)}")
                return

            if (time.time() - started_at) >= (timeout_s + 10.0):
                if not first_draw:
                    _lwrite("\n")
                    sys.stdout.flush()
                try:
                    client.wireless_pair_cancel()
                except Exception:
                    pass
                time.sleep(0.6)
                final_st = client.get_wireless_status_info()
                _print_wireless_status(final_st)
                print(f"配对流程已进入终态：{_pair_result_text(final_st.pair_result)}")
                return
        except Exception as e:
            last_err = f"(轮询异常: {e})"
            _render(last_line, progress_text, last_err)
            # 查询异常时短暂等待，避免忙等打满 CPU。
            time.sleep(0.05)


def _decode_button_mask(mask_value: int) -> str:
    """
    将手柄按键位图解析为可读文本。
    第三位字段按 uint16 位图处理，显示为 hex 与按下 bit 列表。
    约定：按键位为低电平按下（active-low），即 bit=0 表示按下。
    默认展示 B0~B4 五个基础键位；plus/rf 机型可额外启用扩展键位解析。
    """
    return _decode_button_mask_with_profile(mask_value, extended_keys=False)


def _decode_button_mask_with_profile(mask_value: int, extended_keys: bool = False) -> str:
    """
    按机型按键配置解析位图：
    - 基础键位：bit0~bit4 -> B0~B4（active-low）
    - 扩展键位（plus/rf）：bit8~bit12 -> X0~X4（active-low）
    """
    mask = mask_value & 0xFFFF
    pressed = [f"B{bit}" for bit in range(5) if ((mask >> bit) & 0x01) == 0]
    if extended_keys:
        pressed.extend([f"X{bit}" for bit in range(5) if ((mask >> (8 + bit)) & 0x01) == 0])
    if not pressed:
        return f"0x{mask:04X}(-)"
    return f"0x{mask:04X}({','.join(pressed)})"


def _decode_toggle_switch(mask_value: int) -> str:
    """
    解析手柄拨动开关（bit5）。
    约定：bit5=1 表示拨动开关按下/置位，bit5=0 表示释放。
    """
    mask = mask_value & 0xFFFF
    is_on = ((mask >> 5) & 0x01) == 1
    return "ON" if is_on else "OFF"


def _format_joystick(label: str, values: List[int], extended_keys: bool = False) -> str:
    """
    手柄字段可视化：
    [axis0, axis1, key_mask, axis3] -> 轴值 + 按键位图
    """
    if len(values) < 4:
        return f"{label}{values}"
    mask = values[2]
    key_desc = _decode_button_mask_with_profile(mask, extended_keys=extended_keys)
    toggle_desc = _decode_toggle_switch(mask)
    return (
        f"{label}[raw={values}] "
        f"X={values[0]},Y={values[1]} "
        f"keys={key_desc} "
        f"toggle={toggle_desc} "
        f"trigger={values[3]}"
    )


def _print_wireless_status(info: WirelessStatusInfo) -> None:
    print(
        "Wireless 状态:",
        f"status=0x{info.status_code:02X}",
        f"busy={info.pairing_busy}",
        f"step={info.pair_step}",
        f"result={_pair_result_text(info.pair_result)}",
        f"in_config={info.in_config_mode}",
        f"link={info.link_state}",
        f"passthrough={info.passthrough_enabled}",
        f"push={info.push_freq_option}",
        f"local={info.local_device_name or '-'}",
        f"peer={info.paired_device_name or '-'}",
    )


def _wireless_status_line(info: WirelessStatusInfo) -> str:
    return (
        f"Wireless 状态: status=0x{info.status_code:02X} "
        f"busy={info.pairing_busy} step={info.pair_step} "
        f"result={_pair_result_text(info.pair_result)} in_config={info.in_config_mode} "
        f"link={info.link_state} passthrough={info.passthrough_enabled} "
        f"push={info.push_freq_option} local={info.local_device_name or '-'} "
        f"peer={info.paired_device_name or '-'}"
    )


def _print_wireless_command_result(label: str, result: WirelessCommandResult) -> None:
    print(f"{label}: status=0x{result.status_code:02X}({_status_text(result.status_code)})")


def _print_wireless_pair_wait_result(result: WirelessPairWaitResult) -> None:
    print(
        f"配对流程: accept=0x{result.accepted_status:02X} "
        f"elapsed={result.elapsed_s:.2f}s"
    )
    _print_wireless_status(result.final_status)


class _TerminalCbreak:
    """CLI 专用：将终端切换为 cbreak 模式，支持按键无需回车。"""

    def __init__(self) -> None:
        self._enabled = False
        self._fd = None
        self._old = None

    def __enter__(self):
        if os.name != "nt":
            import termios
            import tty

            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._enabled = True
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._enabled and self._fd is not None and self._old is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        self._enabled = False


def _read_key_nonblock() -> Optional[str]:
    """CLI 专用：非阻塞读取单个字符，无输入返回 None。"""
    if os.name == "nt":
        import msvcrt

        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            return ch
        return None
    r, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not r:
        return None
    return sys.stdin.read(1)


def _read_command(exit_event: threading.Event, input_state: Dict[str, str]) -> Optional[str]:
    """CLI 专用：自定义行输入，支持按 q 直接退出。"""
    buf: List[str] = []
    prompt = "qnbot> "
    input_state["buf"] = ""
    input_state["prompt_active"] = True
    sys.stdout.write(prompt)
    sys.stdout.flush()
    while not exit_event.is_set():
        ch = _read_key_nonblock()
        if ch is None:
            time.sleep(0.02)
            continue
        if ch in ("\n", "\r"):
            sys.stdout.write("\n")
            sys.stdout.flush()
            line = "".join(buf).strip()
            input_state["buf"] = ""
            input_state["prompt_active"] = False
            return line
        if ch in ("\x03",):  # Ctrl+C
            exit_event.set()
            input_state["prompt_active"] = False
            return None
        if ch.lower() == "q":
            exit_event.set()
            input_state["prompt_active"] = False
            return None
        if ch in ("\x7f", "\b"):  # Backspace
            if buf:
                buf.pop()
                sys.stdout.write("\b \b")
                sys.stdout.flush()
                input_state["buf"] = "".join(buf)
            continue
        # 可打印字符
        if " " <= ch <= "~":
            buf.append(ch)
            sys.stdout.write(ch)
            sys.stdout.flush()
            input_state["buf"] = "".join(buf)
    return None


def _confirm_or_abort(prompt: str, input_state: Dict[str, str], allow_enter: bool = True) -> bool:
    if prompt:
        print(prompt)
    while True:
        ch = _read_key_nonblock()
        if ch is None:
            time.sleep(0.02)
            continue
        if ch.lower() == "q":
            return False
        if allow_enter and ch in ("\n", "\r"):
            return True


# -----------------------------
# CLI 主流程
# -----------------------------


def _run_interactive(
    client: QnbotClient,
    telemetry_print: List[bool],
    telemetry_style: List[str],
    joystick_extended_keys: List[bool],
    debug_enabled: bool,
    exit_event: threading.Event,
    input_state: Dict[str, str],
    telemetry_lock: threading.Lock,
    telemetry_state: Dict[str, object],
    runtime_support: Optional[Dict[str, bool]] = None,
) -> None:
    _print_help(runtime_support)
    while not exit_event.is_set():
        line = _read_command(exit_event, input_state)
        if exit_event.is_set():
            break
        if not line:
            continue
        cmd = line.lower()
        parts = line.split()

        if cmd in ("help", "?"):
            _print_help(runtime_support)
            continue
        if cmd in ("exit", "quit"):
            exit_event.set()
            break

        try:
            if parts[0].lower() == "lang":
                if len(parts) != 2:
                    print(f"当前语言: {_CLI_LANG}")
                    print("用法: lang <auto|zh-CN|en-US>")
                    continue
                lang = _normalize_cli_lang_token(parts[1])
                if lang is None:
                    print("用法: lang <auto|zh-CN|en-US>")
                    continue
                _set_cli_language(lang)
                print(f"CLI 语言已切换为: {_CLI_LANG}")
            elif cmd == "sys.version":
                _print_system_info(client.get_system_version())
            elif cmd == "sys.status":
                _print_system_status(client.get_system_status())
            elif cmd == "sys.caps":
                _print_caps(client.get_system_capabilities())
            elif cmd == "sys.dfu":
                status = client.enter_system_dfu()
                print(f"System.EnterDfu: status=0x{status:02X}")
            elif parts[0].lower() == "tm.stream":
                if runtime_support is not None and not runtime_support.get("telemetry_runtime", True):
                    print("当前设备不支持 tm.stream 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 3:
                    print("用法: tm.stream <stream_id> <0|1>，stream_id=0xF1/0x00")
                    continue
                res = client.set_telemetry_stream_runtime_config(
                    _parse_stream_id(parts[1]),
                    _parse_int(parts[2]),
                )
                _print_telemetry_runtime_result(res)
            elif parts[0].lower() == "dev.topo":
                if runtime_support is not None and not runtime_support.get("device_profile", True):
                    print("当前设备不支持 dev.* 命令（请使用 help 查看可用命令）。")
                    continue
                _print_device_topology(client.get_device_profile_topology())
            elif parts[0].lower() == "dev.profiles":
                if runtime_support is not None and not runtime_support.get("device_profile", True):
                    print("当前设备不支持 dev.* 命令（请使用 help 查看可用命令）。")
                    continue
                _print_device_profiles(client.get_device_profile_data_profiles())
            elif parts[0].lower() == "telemetry":
                if len(parts) == 3 and parts[1].lower() == "mode":
                    mode = parts[2].lower()
                    if mode not in ("line", "stream"):
                        print("用法: telemetry mode line|stream")
                        continue
                    telemetry_style[0] = mode
                    print(f"遥测输出模式已切换为: {telemetry_style[0]}")
                    continue
                if len(parts) != 2 or parts[1].lower() not in ("on", "off"):
                    print("用法: telemetry on|off")
                    print("或  : telemetry mode line|stream")
                    continue
                sub = parts[1].lower()
                telemetry_print[0] = sub == "on"
                print(f"遥测打印已{'开启' if telemetry_print[0] else '关闭'}")
                if telemetry_print[0]:
                    print("提示：按 q 直接退出（无需回车）")
            elif parts[0].lower() == "enc.list":
                _print_encoder_list(client.get_encoder_info_list())
            elif parts[0].lower() == "enc.info":
                if len(parts) != 2:
                    print("用法: enc.info <ch>")
                    continue
                _print_encoder_info(client.get_encoder_info(_parse_int(parts[1])))
            elif parts[0].lower() == "enc.zero.get":
                if len(parts) != 2:
                    print("用法: enc.zero.get <ch>")
                    continue
                res: EncoderZeroValue = client.get_encoder_zero_value(_parse_int(parts[1]))
                print(
                    f"编码器零位: status=0x{res.status_code:02X} "
                    f"ch={res.channel_index} zero={res.zero_value}"
                )
            elif parts[0].lower() == "enc.zero.set_here":
                if len(parts) != 2:
                    print("用法: enc.zero.set_here <ch>")
                    continue
                res: EncoderSetZeroHereResult = client.set_encoder_zero_here(_parse_int(parts[1]))
                if res.angle_raw is None:
                    print(
                        f"设零位: status=0x{res.status_code:02X} "
                        f"ch={res.channel_index} zero={res.zero_offset}"
                    )
                else:
                    print(
                        f"设零位: status=0x{res.status_code:02X} "
                        f"ch={res.channel_index} zero={res.zero_offset} angle={res.angle_raw}"
                    )
            elif parts[0].lower() == "enc.zero.set":
                if len(parts) != 3:
                    print("用法: enc.zero.set <ch> <zero_value>")
                    continue
                res: EncoderSetZeroValueResult = client.set_encoder_zero_value(
                    _parse_int(parts[1]), _parse_int(parts[2])
                )
                print(
                    f"设置零位: status=0x{res.status_code:02X} ch={res.channel_index}"
                )
            elif parts[0].lower() == "hand.list":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                _print_hand_list(client.get_hand_info_list())
            elif parts[0].lower() == "hand.info":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 2:
                    print("用法: hand.info <idx>")
                    continue
                _print_hand_info(client.get_hand_info(_parse_int(parts[1])))
            elif parts[0].lower() == "hand.calib.get":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 2:
                    print("用法: hand.calib.get <idx>")
                    continue
                _print_hand_calib_params(client.get_hand_calib_params(_parse_int(parts[1])))
            elif parts[0].lower() == "hand.calib.start":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 2:
                    print("用法: hand.calib.start <idx>")
                    continue
                _print_hand_command_result("Hand.CalibStart", client.hand_calib_start(_parse_int(parts[1])))
            elif parts[0].lower() == "hand.calib.commit":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 3:
                    print("用法: hand.calib.commit <idx> <point>")
                    continue
                _print_hand_commit_result(
                    client.hand_calib_commit(_parse_int(parts[1]), _parse_hand_calib_point(parts[2]))
                )
            elif parts[0].lower() == "hand.calib.finish":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 2:
                    print("用法: hand.calib.finish <idx>")
                    continue
                _print_hand_command_result("Hand.CalibFinish", client.hand_calib_finish(_parse_int(parts[1])))
            elif parts[0].lower() == "hand.calib.wizard":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                hand_idx = _resolve_hand_index(client, parts[1] if len(parts) >= 2 else None)
                _run_hand_calib_wizard(
                    client,
                    hand_idx,
                    telemetry_lock,
                    telemetry_state,
                    input_state,
                    telemetry_print,
                    telemetry_style,
                    joystick_extended_keys,
                )
            elif parts[0].lower() == "hand.out.get":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 2:
                    print("用法: hand.out.get <idx>")
                    continue
                _print_hand_calib_params(client.get_hand_calib_params(_parse_int(parts[1])))
            elif parts[0].lower() == "hand.out.set":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 3:
                    print("用法: hand.out.set <idx> <0|1>")
                    continue
                _print_hand_set_output_result(
                    client.hand_set_calib_output(_parse_int(parts[1]), _parse_int(parts[2]))
                )
            elif parts[0].lower() == "hand.param.set":
                if runtime_support is not None and not runtime_support.get("hand", True):
                    print("当前设备不支持 hand.* 命令（请使用 help 查看可用命令）。")
                    continue
                if len(parts) != 4:
                    print("用法: hand.param.set <idx> <point> <value>")
                    continue
                _print_hand_set_param_result(
                    client.hand_set_calib_param(
                        _parse_int(parts[1]),
                        _parse_hand_calib_point(parts[2]),
                        _parse_int(parts[3]),
                    )
                )
            elif parts[0].lower() == "imu.list":
                _print_imu_list(client.get_imu_info_list())
            elif parts[0].lower() == "imu.info":
                if len(parts) != 2:
                    print("用法: imu.info <idx>")
                    continue
                _print_imu_info(client.get_imu_info(_parse_int(parts[1])))
            elif parts[0].lower() == "imu.mag.start":
                if len(parts) != 2:
                    print("用法: imu.mag.start <idx>")
                    continue
                res: ImuMagCalibrateResult = client.imu_mag_calibrate(_parse_int(parts[1]), 0x01)
                print(
                    f"开始校准: status=0x{res.status_code:02X} "
                    f"idx={res.imu_index} state={res.state} err=0x{res.error_code:04X}"
                )
                print("操作提示：请缓慢分别绕 X / Y / Z 三个轴旋转设备，让 IMU 在各个方向采集到磁场数据，完成后使用指令结束校准并自动保存参数。")
            elif parts[0].lower() == "imu.mag.finish":
                if len(parts) != 2:
                    print("用法: imu.mag.finish <idx>")
                    continue
                res: ImuMagCalibrateResult = client.imu_mag_calibrate(_parse_int(parts[1]), 0x02)
                print(
                    f"结束校准: status=0x{res.status_code:02X} "
                    f"idx={res.imu_index} state={res.state} err=0x{res.error_code:04X}"
                )
                print("操作提示：已结束校准并保存数据。")
            elif parts[0].lower() == "imu.mag.wizard":
                imu_idx = _resolve_imu_index(client, parts[1] if len(parts) >= 2 else None)
                _run_imu_mag_wizard(client, imu_idx, input_state)
            elif parts[0].lower() == "haptics.wizard":
                if len(parts) != 1:
                    print("用法: haptics.wizard")
                    continue
                _run_haptics_wizard(
                    client,
                    input_state,
                    telemetry_lock,
                    telemetry_state,
                    runtime_support,
                )
            elif parts[0].lower() == "haptics.force.test":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) < 3 or len(parts) > 6:
                    print("用法: haptics.force.test <idx> <pressure> [mode] [timeout_ms] [hold_s]")
                    continue
                handset_index = _parse_int(parts[1])
                pressure = _parse_int(parts[2])
                mode = _parse_int(parts[3]) if len(parts) >= 4 else 0
                timeout_ms = _parse_int(parts[4]) if len(parts) >= 5 else 2000
                hold_s = float(parts[5]) if len(parts) >= 6 else 0.0
                _run_haptics_force_test(client, handset_index, pressure, mode, timeout_ms, hold_s)
            elif parts[0].lower() == "haptics.trigger.sim":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) < 2 or len(parts) > 4:
                    print("用法: haptics.trigger.sim <idx> [threshold] [poll_ms]")
                    continue
                handset_index = _parse_int(parts[1])
                threshold = _parse_int(parts[2]) if len(parts) >= 3 else 500
                poll_ms = _parse_int(parts[3]) if len(parts) >= 4 else 50
                _run_haptics_trigger_sim(
                    client,
                    handset_index,
                    threshold,
                    poll_ms,
                    telemetry_lock,
                    telemetry_state,
                )
            elif parts[0].lower() == "imu.output.get":
                if runtime_support is not None and not runtime_support.get("imu_output", True):
                    print(_runtime_unsupported_message("imu.output.*"))
                    continue
                if len(parts) != 1:
                    print("用法: imu.output.get")
                    continue
                res = client.get_imu_output_enable()
                _print_imu_output_enable("IMU 输出开关", res)
            elif parts[0].lower() == "imu.output.set":
                if runtime_support is not None and not runtime_support.get("imu_output", True):
                    print(_runtime_unsupported_message("imu.output.*"))
                    continue
                if len(parts) not in (2, 3):
                    print("用法: imu.output.set <0|1> [persist]，persist 默认 1")
                    continue
                persist = _parse_int(parts[2]) if len(parts) == 3 else 1
                res = client.set_imu_output_enable(
                    _parse_int(parts[1]),
                    persist,
                )
                _print_imu_output_enable("IMU 设置输出开关", res)
            elif parts[0].lower() == "wl.status":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                _print_wireless_status(client.get_wireless_status_info())
            elif parts[0].lower() == "wl.pair.start":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                role_mode = None
                if len(parts) >= 2:
                    role_mode = _pair_role_value(parts[1])
                res = client.wireless_pair_start(role_mode=role_mode)
                _print_wireless_command_result("Wireless.PairStart", res)
                print("提示：该回包仅表示已受理，最终结果请用 wl.status 或 wl.pair.wait 查看。")
            elif parts[0].lower() == "wl.pair.wait":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                timeout_s = float(parts[1]) if len(parts) >= 2 else 35.0
                status = client.wait_wireless_pair_result(timeout=timeout_s)
                _print_wireless_status(status)
            elif parts[0].lower() == "wl.pair.run":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                role_mode = None
                timeout_s = 35.0
                if len(parts) >= 2:
                    try:
                        timeout_s = float(parts[1])
                    except ValueError:
                        role_mode = _pair_role_value(parts[1])
                if len(parts) >= 3:
                    timeout_s = float(parts[2])
                res = client.wireless_pair_start_and_wait(
                    role_mode=role_mode,
                    timeout=timeout_s,
                )
                _print_wireless_pair_wait_result(res)
            elif parts[0].lower() == "wl.pair.wizard":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                role_mode = None
                timeout_s = 30.0
                if len(parts) >= 2:
                    try:
                        timeout_s = float(parts[1])
                    except ValueError:
                        role_mode = _pair_role_value(parts[1])
                if len(parts) >= 3:
                    timeout_s = float(parts[2])
                _run_wl_pair_wizard(client, role_mode, timeout_s)
            elif parts[0].lower() == "wl.pair.cancel":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                res = client.wireless_pair_cancel()
                _print_wireless_command_result("Wireless.PairCancel", res)
            elif parts[0].lower() == "wl.reset":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                res = client.wireless_reset()
                _print_wireless_command_result("Wireless.Reset", res)
            elif parts[0].lower() == "wl.config.enter":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                res = client.wireless_enter_config()
                _print_wireless_command_result("Wireless.EnterConfig", res)
            elif parts[0].lower() == "wl.config.exit":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                res = client.wireless_exit_config()
                _print_wireless_command_result("Wireless.ExitConfig", res)
            elif parts[0].lower() == "wl.stream":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                if len(parts) != 2:
                    print("用法: wl.stream <enable>，enable=0/1")
                    continue
                res = client.wireless_set_stream_config(_parse_int(parts[1]))
                _print_wireless_command_result("Wireless.SetStreamConfig", res)
            elif parts[0].lower() == "wl.push":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                if len(parts) != 2:
                    print("用法: wl.push <freq>，freq=2/4")
                    continue
                res = client.wireless_set_push_freq(_parse_int(parts[1]))
                _print_wireless_command_result("Wireless.SetPushFreq", res)
            elif parts[0].lower() == "wl.result":
                if runtime_support is not None and not runtime_support.get("wireless", True):
                    print(_runtime_unsupported_message("wl.*"))
                    continue
                if len(parts) < 2:
                    print("用法: wl.result <0..5> [peer_name]")
                    continue
                pair_result = _parse_int(parts[1])
                peer_name = "" if len(parts) < 3 else " ".join(parts[2:])
                res = client.wireless_set_pair_result_info(pair_result, peer_name)
                _print_wireless_command_result("Wireless.SetPairResultInfo", res)
            elif parts[0].lower() == "haptics.out":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 5:
                    print("用法: haptics.out <channel_id> <amplitude> <pattern_id> <duration_ms>")
                    continue
                res: HapticsOutputResult = client.haptics_set_output(
                    _parse_int(parts[1]), _parse_int(parts[2]), _parse_int(parts[3]), _parse_int(parts[4])
                )
                print(
                    f"Haptics.SetOutput: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"channel={res.channel_id}"
                )
            elif parts[0].lower() == "haptics.cal.get":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 2:
                    print("用法: haptics.cal.get <idx>")
                    continue
                res: HapticsDrvCalStatus = client.haptics_drv_get_cal_status(_parse_int(parts[1]))
                print(
                    f"Haptics.DrvGetCalStatus: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} calibrated={res.calibrated}"
                )
            elif parts[0].lower() == "haptics.cal.run":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 2:
                    print("用法: haptics.cal.run <idx>")
                    continue
                res: HapticsDrvCalibrateResult = client.haptics_drv_calibrate(_parse_int(parts[1]))
                print(
                    f"Haptics.DrvCalibrate: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} result={res.result}"
                )
            elif parts[0].lower() == "haptics.play":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 3:
                    print("用法: haptics.play <idx> <effect_id>")
                    continue
                res: HapticsVibratePlayResult = client.haptics_vibrate_play(_parse_int(parts[1]), _parse_int(parts[2]))
                print(
                    f"Haptics.VibratePlay: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} effect={res.effect_id}"
                )
            elif parts[0].lower() == "haptics.stop":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 2:
                    print("用法: haptics.stop <idx>")
                    continue
                res: HapticsVibrateStopResult = client.haptics_vibrate_stop(_parse_int(parts[1]))
                print(
                    f"Haptics.VibrateStop: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index}"
                )
            elif parts[0].lower() == "haptics.rt":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 3:
                    print("用法: haptics.rt <idx> <amplitude>")
                    continue
                res: HapticsVibrateRealtimeResult = client.haptics_vibrate_realtime(_parse_int(parts[1]), _parse_int(parts[2]))
                print(
                    f"Haptics.VibrateRealtime: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} amplitude={res.amplitude}"
                )
            elif parts[0].lower() == "haptics.enable":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 3:
                    print("用法: haptics.enable <idx> <0|1>")
                    continue
                res: HapticsEnableResult = client.haptics_set_enable(_parse_int(parts[1]), _parse_int(parts[2]))
                print(
                    f"Haptics.SetEnable: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} enable={res.enable}"
                )
            elif parts[0].lower() == "haptics.mode":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 3:
                    print("用法: haptics.mode <idx> <mode>")
                    continue
                res: HapticsModeResult = client.haptics_set_mode(_parse_int(parts[1]), _parse_int(parts[2]))
                print(
                    f"Haptics.SetMode: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} mode={res.mode}"
                )
            elif parts[0].lower() == "haptics.pressure":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 3:
                    print("用法: haptics.pressure <idx> <value>")
                    continue
                res: HapticsPressureResult = client.haptics_set_pressure(_parse_int(parts[1]), _parse_int(parts[2]))
                print(
                    f"Haptics.SetPressure: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} pressure={res.pressure}"
                )
            elif parts[0].lower() == "haptics.timeout":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 3:
                    print("用法: haptics.timeout <idx> <timeout_ms>")
                    continue
                res: HapticsTimeoutResult = client.haptics_set_timeout(_parse_int(parts[1]), _parse_int(parts[2]))
                print(
                    f"Haptics.SetTimeout: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} timeout_ms={res.timeout_ms}"
                )
            elif parts[0].lower() == "haptics.intensity":
                if runtime_support is not None and not runtime_support.get("haptics", True):
                    print(_runtime_unsupported_message("haptics.*"))
                    continue
                if len(parts) != 2:
                    print("用法: haptics.intensity <idx>")
                    continue
                res: HapticsIntensityResult = client.haptics_get_intensity(_parse_int(parts[1]))
                print(
                    f"Haptics.GetIntensity: status=0x{res.status_code:02X}({_status_text(res.status_code)}) "
                    f"idx={res.handset_index} intensity={res.intensity}"
                )
            elif parts[0].lower() == "stats":
                print(client.get_stats())
            else:
                print("未知命令，输入 help 查看可用命令。")
        except Exception as e:
            print(f"执行失败: {e}")
            if debug_enabled:
                print("调试提示：已开启 --debug，可查看原始 QnTP 帧输出。")


def cli_main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Qnbot SDK CLI（交互式测试）")
    parser.add_argument("--port", required=True, help="串口路径，例如 /dev/tty.usbmodemXXXX")
    parser.add_argument("--baudrate", type=int, default=2_000_000, help="波特率（USB CDC 实际可忽略）")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="非交互模式下读取遥测的持续时间（秒），0 表示持续直到按 q",
    )
    parser.add_argument(
        "--no-interactive",
        action="store_true",
        help="关闭交互模式，仅按 duration 读取遥测",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="开启 CLI 调试输出（打印原始 QnTP 帧）",
    )
    parser.add_argument(
        "--telemetry-style",
        choices=["line", "stream"],
        default="line",
        help="遥测输出模式：line(行刷新) 或 stream(持续输出)",
    )
    parser.add_argument(
        "--protocol-mode",
        choices=["auto", "legacy", "qntp"],
        default=None,
        help="解析模式（默认 auto）",
    )
    parser.add_argument(
        "--dispatch-queue-size",
        type=int,
        default=2048,
        help="回调分发队列容量（避免回调阻塞接收线程）",
    )
    parser.add_argument(
        "--telemetry-print",
        action="store_true",
        help="交互模式下启动即打印遥测（默认关闭）",
    )
    parser.add_argument(
        "--no-telemetry-print",
        action="store_true",
        help="关闭遥测打印（交互模式下可用 telemetry on|off 打开/关闭）",
    )
    parser.add_argument(
        "--lang",
        choices=["auto", "zh-CN", "en-US"],
        default="auto",
        help="CLI language, default auto-detect from environment",
    )
    args = parser.parse_args()
    _set_cli_language(args.lang)

    if args.no_interactive:
        telemetry_print = [not args.no_telemetry_print]
    else:
        telemetry_print = [args.telemetry_print and (not args.no_telemetry_print)]
    telemetry_style = [args.telemetry_style]
    telemetry_state = {
        "last_rx_time": None,
        "freq_hz": 0.0,
        "window_start": None,
        "window_count": 0,
        "last_render": 0.0,
        "render_interval": 0.1,  # 行刷新节流，避免打印过重影响解码
        "freq_window": 1.0,      # 频率统计窗口（秒）
        "latest_snap": None,
    }
    telemetry_lock = threading.Lock()
    stats_provider = {"fn": None}
    exit_event = threading.Event()
    input_state = {"buf": "", "prompt_active": False}
    debug_enabled = args.debug
    protocol_mode = args.protocol_mode or "auto"
    joystick_extended_keys = [False]

    def debug_print(msg: str) -> None:
        if debug_enabled:
            print(msg)

    def on_telemetry(snap: TelemetrySnapshot) -> None:
        now = time.time()
        with telemetry_lock:
            if telemetry_state["window_start"] is None:
                telemetry_state["window_start"] = now
                telemetry_state["window_count"] = 0
            telemetry_state["window_count"] += 1
            window_dt = now - telemetry_state["window_start"]
            if window_dt >= telemetry_state["freq_window"]:
                telemetry_state["freq_hz"] = telemetry_state["window_count"] / window_dt
                telemetry_state["window_start"] = now
                telemetry_state["window_count"] = 0
            telemetry_state["latest_snap"] = snap

        if not telemetry_print[0]:
            return

        if telemetry_style[0] == "stream":
            # 持续输出：每帧一行
            freq = telemetry_state["freq_hz"]
            text = (
                f"遥测 [{snap.protocol}] 频率≈{freq:.1f}Hz ts={snap.timestamp:.3f} "
                f"{_format_joystick('LJ', snap.joystick_left, extended_keys=joystick_extended_keys[0])} "
                f"{_format_joystick('RJ', snap.joystick_right, extended_keys=joystick_extended_keys[0])} "
                f"EncL={snap.arm_joint_left} EncR={snap.arm_joint_right} "
                f"EncLr={[round(v, 4) for v in snap.arm_joint_left_rad]} "
                f"EncRr={[round(v, 4) for v in snap.arm_joint_right_rad]} "
                f"TorsoAcc={[round(v, 4) for v in snap.torso_acc]} "
                f"TorsoGyro={[round(v, 4) for v in snap.torso_gyro]} "
                f"TorsoQuat={[round(v, 4) for v in snap.torso_quat]} "
                f"ExtraAcc={[round(v, 4) for v in snap.extra_acc]} "
                f"ExtraGyro={[round(v, 4) for v in snap.extra_gyro]} "
                f"ExtraQuat={[round(v, 4) for v in snap.extra_quat]}"
            )
            print(text)

    def _render_line_loop() -> None:
        while not exit_event.is_set():
            if not telemetry_print[0] or telemetry_style[0] != "line":
                time.sleep(0.05)
                continue
            now = time.time()
            if now - telemetry_state["last_render"] < telemetry_state["render_interval"]:
                time.sleep(0.01)
                continue
            telemetry_state["last_render"] = now
            with telemetry_lock:
                snap = telemetry_state["latest_snap"]
                freq = telemetry_state["freq_hz"]
            if snap is None:
                time.sleep(0.05)
                continue
            # 行刷新：清屏后输出完整信息
            _lwrite("\x1b[?25l")
            _lwrite("\x1b[H\x1b[J")
            _lwrite("=== Qnbot Telemetry ===\n")
            _lwrite(f"遥测 [{snap.protocol}] 频率≈{freq:.1f}Hz\n")
            _lwrite(f"时间戳: {snap.timestamp:.3f}\n")
            _lwrite(
                f"左手柄: {_format_joystick('', snap.joystick_left, extended_keys=joystick_extended_keys[0])}\n"
            )
            _lwrite(
                f"右手柄: {_format_joystick('', snap.joystick_right, extended_keys=joystick_extended_keys[0])}\n"
            )
            _lwrite(f"左臂编码器(原始): {snap.arm_joint_left}\n")
            _lwrite(f"右臂编码器(原始): {snap.arm_joint_right}\n")
            _lwrite(f"左臂编码器(弧度): {[round(v, 4) for v in snap.arm_joint_left_rad]}\n")
            _lwrite(f"右臂编码器(弧度): {[round(v, 4) for v in snap.arm_joint_right_rad]}\n")
            _lwrite(f"躯干IMU Acc: {[round(v, 4) for v in snap.torso_acc]}\n")
            _lwrite(f"躯干IMU Gyro: {[round(v, 4) for v in snap.torso_gyro]}\n")
            _lwrite(f"躯干IMU Quat: {[round(v, 4) for v in snap.torso_quat]}\n")
            _lwrite(f"附加IMU Acc: {[round(v, 4) for v in snap.extra_acc]}\n")
            _lwrite(f"附加IMU Gyro: {[round(v, 4) for v in snap.extra_gyro]}\n")
            _lwrite(f"附加IMU Quat: {[round(v, 4) for v in snap.extra_quat]}\n")
            _lwrite("提示：按 q 直接退出（无需回车）\n")
            if stats_provider.get("fn"):
                stats = stats_provider["fn"]()
                p = stats.get("parser", {})
                b = stats.get("bytes_read", {})
                d = stats.get("dispatch", {})
                _lwrite(
                    f"统计: bytes={b.get('total', 0)} "
                    f"legacy_ok={p.get('legacy_ok', 0)} qntp_ok={p.get('qntp_ok', 0)} "
                    f"drop={p.get('dropped_bytes', 0)} resync={p.get('resyncs', 0)} "
                    f"unk={p.get('unknown_sof', 0)} tail={p.get('legacy_bad_tail', 0)} "
                    f"crc_fail={p.get('qntp_crc_fail', 0)} chk_fail={p.get('legacy_checksum_fail', 0)} "
                    f"len_sw={p.get('legacy_len_switch', 0)} "
                    f"dq={d.get('queue', 0)} dd={d.get('dropped', 0)}\n"
                )
            # 如果命令行有输入内容或正在提示符模式，显示出来
            if input_state.get("prompt_active") or input_state.get("buf"):
                _lwrite(f"qnbot> {input_state.get('buf', '')}\n")
            _lwrite("\x1b[?25h")
            sys.stdout.flush()

    def on_notify(msg: QnTPFrame) -> None:
        if debug_enabled:
            debug_print(
                f"[QnTP] Notify class=0x{msg.msg_class:02X} id=0x{msg.msg_id:02X} "
                f"seq={msg.seq} len={len(msg.payload)} payload={msg.payload.hex()}"
            )

    def on_error(e: Exception) -> None:
        msg = str(e)
        print(f"串口打开失败: {msg}")
        print("提示：端口可能被占用或不存在。请确认没有其它程序占用该串口。")

    render_thread = threading.Thread(target=_render_line_loop, daemon=True)
    render_thread.start()

    try:
        with QnbotClient(
            port=args.port,
            baudrate=args.baudrate,
            protocol_mode=protocol_mode,
            dispatch_queue_size=args.dispatch_queue_size,
            on_error=on_error,
        ) as client:
            stats_provider["fn"] = client.get_stats
            client.register_telemetry_callback(on_telemetry)
            client.register_notify_callback(on_notify)
            if debug_enabled:
                # CLI 调试：打印所有 QnTP 收发帧
                def _dbg_frame(frame: QnTPFrame) -> None:
                    debug_print(
                        f"[QnTP] type=0x{frame.msg_type:02X} class=0x{frame.msg_class:02X} "
                        f"id=0x{frame.msg_id:02X} seq={frame.seq} len={len(frame.payload)} "
                        f"payload={frame.payload.hex()}"
                    )
                client.register_qntp_callback(_dbg_frame)
                if client._serial is not None:
                    _orig_write = client._serial.write

                    def _dbg_write(data: bytes) -> int:
                        debug_print(f"[QnTP] TX: {data.hex(' ')}")
                        return _orig_write(data)

                    client._serial.write = _dbg_write  # type: ignore

            # 系统信息（QnTP）
            runtime_support: Optional[Dict[str, bool]] = None
            try:
                info = client.get_system_version(timeout=2.0)
                _print_system_info(info)
                joystick_extended_keys[0] = info.device_type in (DEVICE_TYPE_EXO_PLUS, DEVICE_TYPE_EXO_PLUS_WIRELESS)
                caps = client.get_system_capabilities(timeout=2.0)
                runtime_support = _build_runtime_support(info.device_type, caps)
                print(
                    "设备识别:",
                    f"type=0x{info.device_type:04X}({_DEVICE_TYPE_NAME.get(info.device_type, 'unknown')})",
                    f"wireless={'on' if runtime_support.get('wireless', True) else 'off'}",
                    f"imu.output={'on' if runtime_support.get('imu_output', True) else 'off'}",
                    f"ext.keys={'on' if joystick_extended_keys[0] else 'off'}",
                )
            except Exception as e:
                print(f"System.GetVersion 失败: {e}")

            if args.no_interactive:
                try:
                    if args.duration <= 0:
                        with _TerminalCbreak():
                            while not exit_event.is_set():
                                ch = _read_key_nonblock()
                                if ch and ch.lower() == "q":
                                    exit_event.set()
                                    break
                                time.sleep(0.05)
                    else:
                        t0 = time.time()
                        with _TerminalCbreak():
                            while time.time() - t0 < args.duration and not exit_event.is_set():
                                ch = _read_key_nonblock()
                                if ch and ch.lower() == "q":
                                    exit_event.set()
                                    break
                                time.sleep(0.05)
                except KeyboardInterrupt:
                    pass
            else:
                if telemetry_print[0]:
                    print("提示：按 q 直接退出（无需回车）")
                with _TerminalCbreak():
                    _run_interactive(
                        client,
                        telemetry_print,
                        telemetry_style,
                        joystick_extended_keys,
                        debug_enabled,
                        exit_event,
                        input_state,
                        telemetry_lock,
                        telemetry_state,
                        runtime_support,
                    )
    except Exception:
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(cli_main())
