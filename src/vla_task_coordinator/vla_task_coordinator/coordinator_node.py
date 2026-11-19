from __future__ import annotations

from cbf_safety_interfaces.msg import SafetyStatus
from cbf_safety_interfaces.srv import InferAction
from geometry_msgs.msg import Pose, TwistStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import JointState

from .policy_core import PolicyConfig, ScriptedWaypointPolicy


class VlaTaskCoordinatorNode(LifecycleNode):
    """Lifecycle coordinator publishing desired TCP twist at policy rate."""

    def __init__(self) -> None:
        super().__init__("vla_task_coordinator")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("v_nom", 0.35)
        self.declare_parameter("a_max", 2.0)
        self.declare_parameter("k_approach", 4.0)
        self.declare_parameter("use_vla_service", False)
        self.declare_parameter("instruction", "pick and place safely")

        self._policy: ScriptedWaypointPolicy | None = None
        self._twist_pub = None
        self._status_sub = None
        self._joint_sub = None
        self._vla_client = None
        self._timer = None
        self._vla_future = None
        self._active = False
        self._emergency_stop = False
        self._tcp_pos = np.array([0.30, -0.15, 0.35], dtype=float)
        self._dt = 0.05

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)
        cfg = PolicyConfig(
            v_nom=float(self.get_parameter("v_nom").value),
            a_max=float(self.get_parameter("a_max").value),
            k_approach=float(self.get_parameter("k_approach").value),
        )
        self._policy = ScriptedWaypointPolicy(cfg)
        self._twist_pub = self.create_lifecycle_publisher(TwistStamped, "desired_tcp_twist", 10)
        self._status_sub = self.create_subscription(SafetyStatus, "safety_status", self._on_safety_status, 10)
        self._joint_sub = self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        if bool(self.get_parameter("use_vla_service").value):
            self._vla_client = self.create_client(InferAction, "infer_action")
        self._timer = self.create_timer(self._dt, self._on_tick)
        self.get_logger().info("Configured vla_task_coordinator")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._policy is not None:
            self._policy.reset()
        self._emergency_stop = False
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._active = False
        self._publish_zero_twist()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._timer is not None:
            self.destroy_timer(self._timer)
        for handle in (self._status_sub, self._joint_sub, self._twist_pub, self._vla_client):
            if handle is not None:
                self.destroy_entity(handle)
        self._timer = self._status_sub = self._joint_sub = self._twist_pub = self._vla_client = None
        self._policy = None
        return TransitionCallbackReturn.SUCCESS

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        self._emergency_stop = bool(msg.emergency_stop)

    def _on_joint_state(self, msg: JointState) -> None:
        # Placeholder hook for external TCP state feedback if provided via FK later.
        _ = msg

    def _on_tick(self) -> None:
        if not self._active or self._policy is None or self._twist_pub is None:
            return
        if self._emergency_stop or self._policy.is_done(self._tcp_pos):
            self._publish_zero_twist()
            return
        if self._vla_client is not None and self._vla_client.service_is_ready():
            self._tick_vla_service()
            return
        v_lin = self._policy.act(self._tcp_pos, self._dt)
        self._tcp_pos = self._tcp_pos + v_lin * self._dt
        self._publish_twist(v_lin)

    def _tick_vla_service(self) -> None:
        if self._vla_future is not None:
            if not self._vla_future.done():
                return
            resp = self._vla_future.result()
            self._vla_future = None
            if resp is not None and resp.ok:
                v_lin = np.array([resp.desired_twist.linear.x, resp.desired_twist.linear.y, resp.desired_twist.linear.z])
                self._publish_twist(v_lin)
            else:
                self._publish_zero_twist()
            return
        req = InferAction.Request()
        req.instruction = str(self.get_parameter("instruction").value)
        req.tcp_pose = Pose()
        req.tcp_pose.position.x, req.tcp_pose.position.y, req.tcp_pose.position.z = self._tcp_pos.tolist()
        self._vla_future = self._vla_client.call_async(req)

    def _publish_zero_twist(self) -> None:
        self._publish_twist(np.zeros(3, dtype=float))

    def _publish_twist(self, v_lin: np.ndarray) -> None:
        if self._twist_pub is None or not self._twist_pub.is_activated:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(v_lin[0])
        msg.twist.linear.y = float(v_lin[1])
        msg.twist.linear.z = float(v_lin[2])
        self._twist_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlaTaskCoordinatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
