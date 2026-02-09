import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class StereoVideoPublisher(Node):
    """
    Publishes frames from two mp4 videos (left/right) as ROS2 Image topics.

    Topics:
      /camera/left/image_raw
      /camera/right/image_raw
    """

    def __init__(self):
        super().__init__("stereo_video_publisher")

        # ---- CHANGE THESE PATHS to your Pair1 files ----
        self.declare_parameter("left_video", "")
        self.declare_parameter("right_video", "")
        self.declare_parameter("rate_hz", 10.0)   # publish rate (not necessarily video fps)
        self.declare_parameter("start_frame", 0)  # starting frame index

        left_path = self.get_parameter("left_video").get_parameter_value().string_value
        right_path = self.get_parameter("right_video").get_parameter_value().string_value
        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.start_frame = int(self.get_parameter("start_frame").get_parameter_value().integer_value)

        if not left_path or not right_path:
            raise RuntimeError("You must pass left_video and right_video parameters.")

        self.left_path = str(Path(left_path).expanduser())
        self.right_path = str(Path(right_path).expanduser())

        self.bridge = CvBridge()
        self.pub_left = self.create_publisher(Image, "/camera/left/image_raw", 10)
        self.pub_right = self.create_publisher(Image, "/camera/right/image_raw", 10)

        self.capL = cv2.VideoCapture(self.left_path)
        self.capR = cv2.VideoCapture(self.right_path)

        if not self.capL.isOpened():
            raise RuntimeError(f"Cannot open left video: {self.left_path}")
        if not self.capR.isOpened():
            raise RuntimeError(f"Cannot open right video: {self.right_path}")

        # Jump to start frame
        if self.start_frame > 0:
            self.capL.set(cv2.CAP_PROP_POS_FRAMES, self.start_frame)
            self.capR.set(cv2.CAP_PROP_POS_FRAMES, self.start_frame)

        self.dt = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(self.dt, self.tick)

        self.frame_idx = self.start_frame
        self.get_logger().info("StereoVideoPublisher started")
        self.get_logger().info(f"Left:  {self.left_path}")
        self.get_logger().info(f"Right: {self.right_path}")
        self.get_logger().info(f"Publishing at {self.rate_hz:.2f} Hz starting frame {self.start_frame}")

    def tick(self):
        okL, frameL = self.capL.read()
        okR, frameR = self.capR.read()

        if not okL or not okR:
            self.get_logger().info("End of one of the videos. Stopping.")
            rclpy.shutdown()
            return

        # ROS Image wants consistent encoding; use bgr8 from OpenCV
        msgL = self.bridge.cv2_to_imgmsg(frameL, encoding="bgr8")
        msgR = self.bridge.cv2_to_imgmsg(frameR, encoding="bgr8")

        # Stamp + frame_id
        now = self.get_clock().now().to_msg()
        msgL.header.stamp = now
        msgR.header.stamp = now
        msgL.header.frame_id = "left_camera"
        msgR.header.frame_id = "right_camera"

        self.pub_left.publish(msgL)
        self.pub_right.publish(msgR)

        if self.frame_idx % 30 == 0:
            self.get_logger().info(f"Published frame {self.frame_idx}")

        self.frame_idx += 1


def main():
    rclpy.init()
    node = StereoVideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.capL.release()
            node.capR.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
