from __future__ import annotations

import multiprocessing as mp
import time
from typing import Sequence

from qnbot_sdk import TelemetrySnapshot

from ._client_surface import TelemetryClientSurface, install_qnbot_client_surface

DEFAULT_JOINT_COUNT = 8
DEFAULT_BUTTON_COUNT = 10


def _make_key_mask(buttons: Sequence[int], switch_on: int) -> int:
    mask = 0
    for index in range(5):
        if not buttons[index]:
            mask |= 1 << index
    if switch_on:
        mask |= 1 << 5
    for index in range(5):
        if not buttons[5 + index]:
            mask |= 1 << (8 + index)
    return mask


def _arm_raw_values(joint_values_rad: Sequence[float]) -> list[int]:
    return [int(round(value * 1000.0)) for value in joint_values_rad]


@install_qnbot_client_surface
class FakeClient(TelemetryClientSurface):
    """Manual fake telemetry client driven by a small GUI."""

    def __init__(self, gui: bool = True):
        ctx = mp.get_context("spawn")
        self.gui = bool(gui)
        self._ctx = ctx
        self._running = ctx.Value("b", 0)
        self._closed = ctx.Value("b", 0)
        self._updated_at = ctx.Value("d", time.time())
        self._left_arm_rad = ctx.Array("d", [0.0] * DEFAULT_JOINT_COUNT)
        self._right_arm_rad = ctx.Array("d", [0.0] * DEFAULT_JOINT_COUNT)
        self._left_axis = ctx.Array("i", [0, 0])
        self._right_axis = ctx.Array("i", [0, 0])
        self._left_trigger = ctx.Value("i", 0)
        self._right_trigger = ctx.Value("i", 0)
        self._left_buttons = ctx.Array("b", [0] * DEFAULT_BUTTON_COUNT)
        self._right_buttons = ctx.Array("b", [0] * DEFAULT_BUTTON_COUNT)
        self._left_switch = ctx.Value("b", 0)
        self._right_switch = ctx.Value("b", 0)
        self._gui_process: mp.Process | None = None
        self._latest_telemetry: TelemetrySnapshot | None = None

    def start(self) -> None:
        if self._running.value:
            return
        self._closed.value = 0
        self._running.value = 1
        self._updated_at.value = time.time()
        if self.gui:
            from .fake_client_gui import run_fake_client_gui

            self._gui_process = self._ctx.Process(
                target=run_fake_client_gui,
                args=(self._shared_state(),),
                daemon=True,
            )
            self._gui_process.start()

    def stop(self) -> None:
        self._closed.value = 1
        self._running.value = 0
        if self._gui_process is None:
            return
        self._gui_process.join(timeout=1.0)
        if self._gui_process.is_alive():
            self._gui_process.terminate()
            self._gui_process.join(timeout=1.0)
        self._gui_process = None

    def get_latest_telemetry(self) -> TelemetrySnapshot | None:
        if not self._running.value:
            if self._closed.value:
                return None
            self.start()
        if not self._running.value:
            return None
        snapshot = TelemetrySnapshot(protocol="fake", timestamp=time.time())
        snapshot.arm_joint_left_rad = list(self._left_arm_rad[:])
        snapshot.arm_joint_right_rad = list(self._right_arm_rad[:])
        snapshot.arm_joint_left = _arm_raw_values(snapshot.arm_joint_left_rad)
        snapshot.arm_joint_right = _arm_raw_values(snapshot.arm_joint_right_rad)
        snapshot.joystick_left = [
            int(self._left_axis[0]),
            int(self._left_axis[1]),
            _make_key_mask(self._left_buttons, int(self._left_switch.value)),
            int(self._left_trigger.value),
        ]
        snapshot.joystick_right = [
            int(self._right_axis[0]),
            int(self._right_axis[1]),
            _make_key_mask(self._right_buttons, int(self._right_switch.value)),
            int(self._right_trigger.value),
        ]
        snapshot.torso_quat = [0.0, 0.0, 0.0, 1.0]
        snapshot.extra_quat = [0.0, 0.0, 0.0, 1.0]
        self._latest_telemetry = snapshot
        return snapshot

    def _shared_state(self) -> dict[str, object]:
        return {
            "running": self._running,
            "closed": self._closed,
            "updated_at": self._updated_at,
            "left_arm_rad": self._left_arm_rad,
            "right_arm_rad": self._right_arm_rad,
            "left_axis": self._left_axis,
            "right_axis": self._right_axis,
            "left_trigger": self._left_trigger,
            "right_trigger": self._right_trigger,
            "left_buttons": self._left_buttons,
            "right_buttons": self._right_buttons,
            "left_switch": self._left_switch,
            "right_switch": self._right_switch,
        }
