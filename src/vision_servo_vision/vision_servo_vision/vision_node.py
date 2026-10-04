"""视觉节点：订阅相机，跑 YOLOv11-pose，发布特征 + 可视化."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from vision_servo_interfaces.msg import TargetFeatures

from .yolo_detector import YOLODetector
from .temporal_filter import TemporalFilter
from .feature_extractor import extract_features


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        # 参数
        self.declare_parameter("engine_path", "")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("img_cx", 341.666)
        self.declare_parameter("img_cy", 232.995)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("lost_tolerance", 10)
        self.declare_parameter("infer_interval", 1)
        self.declare_parameter("image_topic", "/ascamera/AsCamera/rgb0/image")
        self.declare_parameter("num_kpts", 4)
        self.declare_parameter("publish_viz", True)

        self.engine_path = self.get_parameter("engine_path").value
        self.conf_threshold = self.get_parameter("conf_threshold").value
        self.img_cx = self.get_parameter("img_cx").value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.infer_interval = self.get_parameter("infer_interval").value
        self.image_topic = self.get_parameter("image_topic").value
        self.num_kpts = self.get_parameter("num_kpts").value
        self.publish_viz = self.get_parameter("publish_viz").value

        # 组件
        self.bridge = CvBridge()
        self.detector = YOLODetector(
            self.engine_path,
            self.conf_threshold,
            num_kpts=self.num_kpts,
        )
        self.filter = TemporalFilter(
            self.ema_alpha,
            self.get_parameter("lost_tolerance").value)

        self.frame_count = 0

        # QoS: 只处理最新帧
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos)

        # 特征发布
        self.pub = self.create_publisher(
            TargetFeatures, "/vision_servo/target_features", 1)

        # ★ 可视化发布
        self.viz_pub = self.create_publisher(
            Image, "/vision_servo/debug_image", 1)

        self.get_logger().info(
            f"Vision node started\n"
            f"  engine: {self.engine_path}\n"
            f"  topic:  {self.image_topic}\n"
            f"  img_cx: {self.img_cx}\n"
            f"  num_kpts: {self.num_kpts}\n"
            f"  publish_viz: {self.publish_viz}")

    def on_image(self, msg):
        self.frame_count += 1
        if self.frame_count % self.infer_interval != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return

        kps = self.detector.infer(frame)

        # ★ 可视化（每帧都发）
        if self.publish_viz:
            self.publish_debug_image(frame, kps, msg.header)

        if kps is None:
            self.filter.on_detection(False)
            if self.filter.is_lost():
                self.publish_invalid(msg.header)
            return

        self.filter.on_detection(True)
        kps = self.filter.update_keypoints(kps)
        center_x, h_left, h_right, h_avg = extract_features(kps, self.img_cx)
        center_x, h_left, h_right, h_avg = self.filter.update_features(
            center_x, h_left, h_right, h_avg)

        out = TargetFeatures()
        out.header = msg.header
        out.center_x = float(center_x)
        out.h_left = float(h_left)
        out.h_right = float(h_right)
        out.h_avg = float(h_avg)
        out.e_yaw = float((h_left - h_right) / max(h_avg, 1.0))
        out.conf = float(self.detector.last_conf)
        out.valid = True
        self.pub.publish(out)

    def publish_debug_image(self, frame, kps, header):
        """在图像上画框和关键点，发布出去."""
        if kps is not None:
            self.get_logger().info(f"[DEBUG] kps={kps.tolist()}")
        else:
            self.get_logger().info("[DEBUG] kps is None")
        vis = frame.copy()

        if kps is not None and len(kps) >= 4:
            # 画 4 个关键点
            colors = [
                (0, 0, 255),      # p0 红
                (0, 255, 0),      # p1 绿
                (0, 255, 255),    # p2 黄
                (255, 0, 0),      # p3 蓝
            ]
            for i in range(4):
                x, y = int(kps[i][0]), int(kps[i][1])
                cv2.circle(vis, (x, y), 8, colors[i], -1)
                cv2.putText(vis, f"p{i}", (x + 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2)

            # 画四边形
            pts = kps[:4].astype(int)
            for i in range(4):
                p1 = tuple(pts[i])
                p2 = tuple(pts[(i + 1) % 4])
                cv2.line(vis, p1, p2, (0, 255, 0), 2)

            # 置信度
            cv2.putText(vis, f"conf={self.detector.last_conf:.2f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 0), 2)
        else:
            cv2.putText(vis, "NO DETECTION",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
            out_msg.header = header
            self.viz_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"publish viz failed: {e}")

    def publish_invalid(self, header):
        out = TargetFeatures()
        out.header = header
        out.valid = False
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()