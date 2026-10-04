"""控制节点：订阅特征，发布 cmd_vel，触发急停."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_msgs.msg import Empty

from vision_servo_interfaces.msg import TargetFeatures, AlignStatus
from .align_controller import AlignController


class ControlNode(Node):
    def __init__(self):
        super().__init__("align_controller")

        # 参数
        self.declare_parameter("img_cx", 341.666)
        self.declare_parameter("h_avg_target", 200.0)
        self.declare_parameter("tol_x", 5.0)
        self.declare_parameter("tol_yaw", 0.02)
        self.declare_parameter("tol_z", 10.0)
        self.declare_parameter("dead_x", 3.0)
        self.declare_parameter("dead_yaw", 0.01)
        self.declare_parameter("dead_z", 5.0)
        self.declare_parameter("kp_x", 0.003)
        self.declare_parameter("kp_yaw", 1.5)
        self.declare_parameter("kp_z", 0.002)
        self.declare_parameter("vmax", 0.15)
        self.declare_parameter("wmax", 0.5)
        self.declare_parameter("aligned_frames", 5)
        self.declare_parameter("servo_timeout", 3.0)
# ═════════════════════════════════════════════════════════════
        # ★ 接近阶段参数（现场调）
        self.declare_parameter("grab_distance_mm", 640.0)
        self.declare_parameter("approach_margin_mm", 100.0)
        self.declare_parameter("approach_vmax", 0.25)
        self.declare_parameter("kp_z_approach", 0.5)

        params = {
            k: self.get_parameter(k).value
            for k in [
                'img_cx', 'h_avg_target',
                'tol_x', 'tol_yaw', 'tol_z',
                'dead_x', 'dead_yaw', 'dead_z',
                'kp_x', 'kp_yaw', 'kp_z',
                'vmax', 'wmax',
                'aligned_frames', 'servo_timeout',
                'grab_distance_mm', 'approach_margin_mm',
                'approach_vmax', 'kp_z_approach',
            ]
        }

        self.ctrl = AlignController(params)

        # 订阅
        self.create_subscription(
            TargetFeatures, "/vision_servo/target_features",
            self.on_features, 1)

        # 启动信号（决策层发）
        self.create_subscription(
            Empty, "/vision_servo/start",
            self.on_start, 1)

        # 发布
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_vision", 1)
        self.stop_pub = self.create_publisher(Bool, "/vision_servo/stop", 1)
        self.event_pub = self.create_publisher(String, "/vision_servo/event", 1)
        self.status_pub = self.create_publisher(AlignStatus, "/vision_servo/status", 1)

        # 控制循环 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

        self.get_logger().info("Control node started")

    def on_features(self, msg):
        self.ctrl.on_features(msg)

    def on_start(self, msg):
        self.get_logger().info("Received /vision_servo/start, entering SERVO")
        self.ctrl.start()

    def control_loop(self):
        cmd, status = self.ctrl.compute_cmd()

        # ── 先处理 status ──
        if status == "ALIGNED":
            self.cmd_pub.publish(Twist())
            self.trigger_aligned()
            return

        if status in ("TIMEOUT", "LOST"):
            self.cmd_pub.publish(Twist())
            self.trigger_failed(status)
            return

        # ── 再检查状态 ──
        if self.ctrl.state == "ALIGNED_STOP":
            self.cmd_pub.publish(Twist())   # 持续发零速度防漂移
            return

        if self.ctrl.state != "SERVO":
            return

        # ── 正常 SERVO 阶段，发速度 ──
        self.cmd_pub.publish(cmd)

    def trigger_aligned(self):
        self.cmd_pub.publish(Twist())
        self.stop_pub.publish(Bool(data=True))
        self.event_pub.publish(String(data="VISUAL_ALIGNED"))

        st = AlignStatus()
        st.status = AlignStatus.STATUS_ALIGNED
        st.message = "aligned"
        self.status_pub.publish(st)
        self.get_logger().info("ALIGNED! Stop triggered.")

    def trigger_failed(self, reason):
        self.cmd_pub.publish(Twist())
        self.stop_pub.publish(Bool(data=True))
        self.event_pub.publish(String(data=f"VISUAL_FAILED:{reason}"))

        st = AlignStatus()
        st.status = AlignStatus.STATUS_FAILED
        st.message = reason
        self.status_pub.publish(st)
        self.get_logger().warn(f"Failed: {reason}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()