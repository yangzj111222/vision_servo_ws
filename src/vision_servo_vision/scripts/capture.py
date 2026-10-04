import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CaptureNode(Node):
    def __init__(self):
        super().__init__("capture")
        self.bridge = CvBridge()
        self.save_dir = os.path.expanduser("~/datasets/cube_pose/images/train")
        os.makedirs(self.save_dir, exist_ok=True)
        self.count = 0
        self.sub = self.create_subscription(
            Image, "/ascamera/AsCamera/rgb0/image", self.on_image, 1)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("capture", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            path = f"{self.save_dir}/img_{self.count:04d}.jpg"
            cv2.imwrite(path, frame)
            self.count += 1
            self.get_logger().info(f"Saved {path} ({self.count})")
        elif key == ord('q'):
            rclpy.shutdown()

def main():
    rclpy.init()
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()