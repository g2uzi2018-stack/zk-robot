from __future__ import annotations

import random
import time
import tkinter as tk
from tkinter import ttk

BUTTON_LABELS = ["B0", "B1", "B2", "B3", "B4", "X0", "X1", "X2", "X3", "X4"]
JOINT_LIMITS = {
    "left": [
        (-2.0944, 2.0944),
        (-2.0944, 2.0944),
        (-2.0944, 2.0944),
        (-1.5707, 0.6629),
        (-2.0944, 2.0944),
        (-1.5707, 1.2211),
        (-0.785, 1.5707),
        (0.0, 0.0),
    ],
    "right": [
        (-2.0944, 2.0944),
        (-2.0944, 2.0944),
        (-2.0944, 2.0944),
        (-1.5707, 0.6629),
        (-2.0944, 2.0944),
        (-1.5707, 1.2211),
        (-1.5707, 0.785),
        (0.0, 0.0),
    ],
}
AXIS_RANGE = (-4096, 4095)
TRIGGER_RANGE = (0, 4095)


def run_fake_client_gui(shared_state: dict[str, object]) -> None:
    root = tk.Tk()
    root.title("Qnbot Fake Client")
    root.geometry("1180x860")
    content = ttk.Frame(root, padding=8)
    content.pack(fill="both", expand=True)

    status_var = tk.StringVar(value="ready")

    def mark_changed() -> None:
        shared_state["updated_at"].value = time.time()
        status_var.set(f"updated {time.strftime('%H:%M:%S')}")

    def set_arm(side: str, joint_index: int, value: str) -> None:
        shared_state[f"{side}_arm_rad"][joint_index] = float(value)
        mark_changed()

    def set_axis(side: str, axis_index: int, value: str) -> None:
        shared_state[f"{side}_axis"][axis_index] = int(float(value))
        mark_changed()

    def set_trigger(side: str, value: str) -> None:
        shared_state[f"{side}_trigger"].value = int(float(value))
        mark_changed()

    def set_button(side: str, button_index: int, var: tk.IntVar) -> None:
        shared_state[f"{side}_buttons"][button_index] = int(var.get())
        mark_changed()

    def set_switch(side: str, var: tk.IntVar) -> None:
        shared_state[f"{side}_switch"].value = int(var.get())
        mark_changed()

    def reset_all() -> None:
        for side in ("left", "right"):
            for joint_index in range(8):
                shared_state[f"{side}_arm_rad"][joint_index] = 0.0
            shared_state[f"{side}_axis"][0] = 0
            shared_state[f"{side}_axis"][1] = 0
            shared_state[f"{side}_trigger"].value = 0
            shared_state[f"{side}_switch"].value = 0
            for button_index in range(len(BUTTON_LABELS)):
                shared_state[f"{side}_buttons"][button_index] = 0
        populate_ui_from_state()
        mark_changed()

    def randomize_joints() -> None:
        for side in ("left", "right"):
            for joint_index, (lower, upper) in enumerate(JOINT_LIMITS[side]):
                if lower == upper:
                    value = lower
                else:
                    value = random.uniform(lower, upper)
                shared_state[f"{side}_arm_rad"][joint_index] = value
        populate_ui_from_state()
        mark_changed()

    def populate_ui_from_state() -> None:
        for side in ("left", "right"):
            for joint_index, var in joint_vars[side].items():
                var.set(float(shared_state[f"{side}_arm_rad"][joint_index]))
            axis_vars[side]["x"].set(int(shared_state[f"{side}_axis"][0]))
            axis_vars[side]["y"].set(int(shared_state[f"{side}_axis"][1]))
            trigger_vars[side].set(int(shared_state[f"{side}_trigger"].value))
            switch_vars[side].set(int(shared_state[f"{side}_switch"].value))
            for button_index, var in button_vars[side].items():
                var.set(int(shared_state[f"{side}_buttons"][button_index]))

    def handle_close() -> None:
        shared_state["closed"].value = 1
        shared_state["running"].value = 0
        root.destroy()

    joint_vars: dict[str, dict[int, tk.DoubleVar]] = {"left": {}, "right": {}}
    axis_vars: dict[str, dict[str, tk.IntVar]] = {
        "left": {"x": tk.IntVar(value=0), "y": tk.IntVar(value=0)},
        "right": {"x": tk.IntVar(value=0), "y": tk.IntVar(value=0)},
    }
    trigger_vars = {"left": tk.IntVar(value=0), "right": tk.IntVar(value=0)}
    switch_vars = {"left": tk.IntVar(value=0), "right": tk.IntVar(value=0)}
    button_vars: dict[str, dict[int, tk.IntVar]] = {"left": {}, "right": {}}

    for column, side in enumerate(("left", "right")):
        column_frame = ttk.Frame(content)
        column_frame.grid(row=0, column=column, sticky="nsew", padx=8, pady=8)
        content.columnconfigure(column, weight=1)
        content.rowconfigure(0, weight=1)

        arm_frame = ttk.LabelFrame(column_frame, text=f"{side.title()} Arm", padding=8)
        arm_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        for joint_index in range(8):
            var = tk.DoubleVar(value=0.0)
            joint_vars[side][joint_index] = var
            ttk.Label(arm_frame, text=f"Joint {joint_index + 1}").grid(
                row=joint_index,
                column=0,
                sticky="w",
                padx=(0, 8),
                pady=4,
            )
            tk.Scale(
                arm_frame,
                from_=JOINT_LIMITS[side][joint_index][0],
                to=JOINT_LIMITS[side][joint_index][1],
                resolution=0.01,
                orient="horizontal",
                length=360,
                variable=var,
                command=lambda value, s=side, i=joint_index: set_arm(s, i, value),
            ).grid(row=joint_index, column=1, sticky="ew", pady=2)
        arm_frame.columnconfigure(1, weight=1)
        hand_frame = ttk.LabelFrame(column_frame, text=f"{side.title()} Handset", padding=8)
        hand_frame.grid(row=1, column=0, sticky="nsew")

        ttk.Label(hand_frame, text="Axis X").grid(row=0, column=0, sticky="w")
        tk.Scale(
            hand_frame,
            from_=AXIS_RANGE[0],
            to=AXIS_RANGE[1],
            orient="horizontal",
            length=300,
            variable=axis_vars[side]["x"],
            command=lambda value, s=side: set_axis(s, 0, value),
        ).grid(row=0, column=1, sticky="ew")

        ttk.Label(hand_frame, text="Axis Y").grid(row=1, column=0, sticky="w")
        tk.Scale(
            hand_frame,
            from_=AXIS_RANGE[0],
            to=AXIS_RANGE[1],
            orient="horizontal",
            length=300,
            variable=axis_vars[side]["y"],
            command=lambda value, s=side: set_axis(s, 1, value),
        ).grid(row=1, column=1, sticky="ew")

        ttk.Label(hand_frame, text="Trigger").grid(row=2, column=0, sticky="w")
        tk.Scale(
            hand_frame,
            from_=TRIGGER_RANGE[0],
            to=TRIGGER_RANGE[1],
            orient="horizontal",
            length=300,
            variable=trigger_vars[side],
            command=lambda value, s=side: set_trigger(s, value),
        ).grid(row=2, column=1, sticky="ew")

        ttk.Checkbutton(
            hand_frame,
            text="Switch ON",
            variable=switch_vars[side],
            command=lambda s=side: set_switch(s, switch_vars[s]),
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 8))

        button_frame = ttk.LabelFrame(hand_frame, text="Buttons", padding=8)
        button_frame.grid(row=4, column=0, columnspan=2, sticky="ew")
        for button_index, label in enumerate(BUTTON_LABELS):
            var = tk.IntVar(value=0)
            button_vars[side][button_index] = var
            ttk.Checkbutton(
                button_frame,
                text=label,
                variable=var,
                command=lambda s=side, i=button_index: set_button(s, i, button_vars[s][i]),
            ).grid(row=button_index // 5, column=button_index % 5, sticky="w", padx=4, pady=2)

        hand_frame.columnconfigure(1, weight=1)
        column_frame.columnconfigure(0, weight=1)

    controls_frame = ttk.Frame(root, padding=(8, 0, 8, 8))
    controls_frame.pack(fill="x")
    ttk.Button(controls_frame, text="All Zero", command=reset_all).pack(side="left")
    ttk.Button(controls_frame, text="Random Joints", command=randomize_joints).pack(side="left", padx=(8, 0))
    ttk.Label(controls_frame, textvariable=status_var).pack(side="right")

    populate_ui_from_state()
    root.protocol("WM_DELETE_WINDOW", handle_close)
    root.mainloop()
