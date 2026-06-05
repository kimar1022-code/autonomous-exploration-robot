#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import time
import threading

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ROS2 Publisher
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.picam2 = None
        self.running = True

        self.init_camera()

        # 캡처 스레드 시작
        self.camera_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.camera_thread.start()

        self.get_logger().info('Camera Node Started')

    def init_camera(self):
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.get_logger().info('Camera initialized')
            time.sleep(2)  # AE/AWB 수렴 대기
        except Exception as e:
            self.get_logger().error(f'Camera init failed: {e}')

    def capture_loop(self):
        if self.picam2 is None:
            return
        while self.running:
            try:
                # picamera2 "RGB888"은 실제로 BGR 순서 → 변환 없이 그대로 사용
                frame = self.picam2.capture_array()
                if frame is not None:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    ros_image.header.frame_id = 'camera_link'
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    self.image_pub.publish(ros_image)
                time.sleep(0.033)  # ~30 FPS
            except Exception as e:
                self.get_logger().warn(f'Capture error: {e}')
                time.sleep(0.1)

    def destroy_node(self):
        self.running = False
        if self.picam2 is not None:
            self.picam2.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
