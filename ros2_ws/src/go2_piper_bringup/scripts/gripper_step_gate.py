#!/usr/bin/env python3
"""Deterministic no-contact gate for the real-time gripper effort loop."""

from __future__ import annotations

import json
import math
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GripperStepGate(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_gripper_step_gate")
        self.positions = None
        self.velocities = None
        self.publisher = self.create_publisher(
            Float64MultiArray, "/go2_piper/gripper/target", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        self.set_entity = self.create_client(
            SetEntityState, "/gazebo/set_entity_state")

    def _joint_state(self, message: JointState) -> None:
        index = {name: value for value, name in enumerate(message.name)}
        if not all(name in index for name in ("joint7", "joint8")):
            return
        self.positions = [message.position[index[name]] for name in ("joint7", "joint8")]
        self.velocities = [message.velocity[index[name]] for name in ("joint7", "joint8")]

    def wait_until_ready(self, timeout: float = 15.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.positions is not None and self.publisher.get_subscription_count() > 0
                    and self.set_entity.wait_for_service(timeout_sec=0.0)):
                return
        raise RuntimeError("gripper gate interfaces did not become ready")

    def move_entity(self, name: str, xyz: list[float]) -> None:
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = name
        request.state.reference_frame = "world"
        request.state.pose.position.x = xyz[0]
        request.state.pose.position.y = xyz[1]
        request.state.pose.position.z = xyz[2]
        request.state.pose.orientation.w = 1.0
        future = self.set_entity.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f"failed to move {name} away from the fingers")

    def clear_workcell(self) -> None:
        self.move_entity("target_box", [5.0, 5.0, 1.0])
        self.move_entity("pickup_table", [5.0, 6.0, 0.496])
        self.move_entity("drop_table", [5.0, 7.0, 0.496])

    def step(self, target: list[float], duration: float = 3.0) -> dict:
        self.publisher.publish(Float64MultiArray(data=target))
        started = time.monotonic()
        samples = []
        while time.monotonic() - started < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.positions is not None:
                samples.append((time.monotonic() - started,
                                self.positions.copy(), self.velocities.copy()))
        tail = [sample for sample in samples if sample[0] >= duration - 0.25]
        if not tail:
            raise RuntimeError("no gripper joint samples in settling window")
        mean_position = [sum(sample[1][i] for sample in tail) / len(tail) for i in range(2)]
        min_position = [min(sample[1][i] for sample in tail) for i in range(2)]
        max_position = [max(sample[1][i] for sample in tail) for i in range(2)]
        position_span = [max_position[i] - min_position[i] for i in range(2)]
        max_tail_velocity = [max(abs(sample[2][i]) for sample in tail) for i in range(2)]
        velocity_sign_agreement = []
        for index in range(2):
            comparable = []
            for previous, current in zip(samples, samples[1:]):
                dt = current[0] - previous[0]
                if dt <= 0.0:
                    continue
                finite_difference = (current[1][index] - previous[1][index]) / dt
                reported = current[2][index]
                if abs(finite_difference) > 0.005 and abs(reported) > 0.005:
                    comparable.append(finite_difference * reported > 0.0)
            velocity_sign_agreement.append(
                sum(comparable) / len(comparable) if comparable else 0.0)
        max_error = max(abs(mean_position[i] - target[i]) for i in range(2))
        max_velocity = max(max_tail_velocity)
        finite = all(math.isfinite(value) for sample in samples
                     for values in sample[1:] for value in values)
        return {
            "target": target,
            "samples": len(samples),
            "mean_position": mean_position,
            "position_span": position_span,
            "max_tail_velocity": max_tail_velocity,
            "velocity_sign_agreement": velocity_sign_agreement,
            "max_position_error": max_error,
            "finite": finite,
            "passed": finite and max_error <= 0.002 and max_velocity <= 0.02,
        }


def main() -> int:
    rclpy.init()
    node = GripperStepGate()
    try:
        node.wait_until_ready()
        node.clear_workcell()
        opened = node.step([0.04, -0.04])
        closed = node.step([0.025, -0.025])
        report = {"open": opened, "close": closed,
                  "passed": opened["passed"] and closed["passed"]}
        print(json.dumps(report, separators=(",", ":")))
        return 0 if report["passed"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
