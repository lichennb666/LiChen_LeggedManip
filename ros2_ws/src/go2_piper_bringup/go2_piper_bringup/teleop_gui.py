#!/usr/bin/env python3
"""X11 keyboard teleop with real press/release handling for mapping."""

from __future__ import annotations

import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


LINEAR_SPEED = 0.20
ANGULAR_SPEED = 0.80
PUBLISH_PERIOD_MS = 50
RELEASE_DEBOUNCE_MS = 80


class TeleopGui(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_teleop_gui")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.active: set[str] = set()
        self.release_jobs: dict[str, str] = {}

        self.root = tk.Tk()
        self.root.title("Go2 Piper Mapping Teleop")
        self.root.geometry("430x260")
        self.root.configure(bg="#151515")
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.root.bind("<KeyPress>", self.on_press)
        self.root.bind("<KeyRelease>", self.on_release)
        self.root.focus_force()

        text = (
            "Click this window, then hold keys\n\n"
            "W/S: forward/backward  0.20 m/s\n"
            "A/D: strafe left/right 0.20 m/s\n"
            "Q/E: turn left/right   0.80 rad/s\n"
            "X or Space: stop\n\n"
            "This window uses KeyPress/KeyRelease,\n"
            "so release stops immediately."
        )
        label = tk.Label(
            self.root,
            text=text,
            justify="left",
            fg="#f2f2f2",
            bg="#151515",
            font=("DejaVu Sans Mono", 12),
            padx=18,
            pady=18,
        )
        label.pack(fill="both", expand=True)
        self.status = tk.Label(
            self.root,
            text="cmd: vx=0.00 vy=0.00 wz=0.00",
            fg="#76d275",
            bg="#151515",
            font=("DejaVu Sans Mono", 11),
        )
        self.status.pack(fill="x", pady=(0, 12))
        self.root.after(PUBLISH_PERIOD_MS, self.tick)

    def on_press(self, event) -> None:
        key = event.keysym.lower()
        char = (event.char or "").lower()
        key = char if char in {"w", "a", "s", "d", "q", "e", "x"} else key
        if key in self.release_jobs:
            self.root.after_cancel(self.release_jobs.pop(key))
        if key in {"space", "x"}:
            self.active.clear()
            self.publish_stop()
            return
        if key in {"w", "a", "s", "d", "q", "e"}:
            self.active.add(key)

    def on_release(self, event) -> None:
        key = event.keysym.lower()
        char = (event.char or "").lower()
        key = char if char in {"w", "a", "s", "d", "q", "e", "x"} else key
        if key in {"w", "a", "s", "d", "q", "e"}:
            if key in self.release_jobs:
                self.root.after_cancel(self.release_jobs.pop(key))
            self.release_jobs[key] = self.root.after(
                RELEASE_DEBOUNCE_MS, lambda key=key: self.finish_release(key))

    def finish_release(self, key: str) -> None:
        self.release_jobs.pop(key, None)
        self.active.discard(key)

    def command(self) -> tuple[float, float, float]:
        vx = 0.0
        vy = 0.0
        wz = 0.0
        if "w" in self.active:
            vx += LINEAR_SPEED
        if "s" in self.active:
            vx -= LINEAR_SPEED
        if "a" in self.active:
            vy += LINEAR_SPEED
        if "d" in self.active:
            vy -= LINEAR_SPEED
        if "q" in self.active:
            wz += ANGULAR_SPEED
        if "e" in self.active:
            wz -= ANGULAR_SPEED
        return vx, vy, wz

    def tick(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        vx, vy, wz = self.command()
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.publisher.publish(msg)
        self.status.configure(text=f"cmd: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
        self.root.after(PUBLISH_PERIOD_MS, self.tick)

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def shutdown(self) -> None:
        self.active.clear()
        self.publish_stop()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = TeleopGui()
    node.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
