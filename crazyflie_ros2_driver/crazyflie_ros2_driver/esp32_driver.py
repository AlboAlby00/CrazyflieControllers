import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import requests
import numpy as np
from cv_bridge import CvBridge

class Esp32Driver(Node):

    def __init__(self):
        super().__init__('esp_32_driver')
        self.get_logger().info("esp 32 camera driver running")
        self.publisher = self.create_publisher(Image, 'esp_32/camera', 10)
        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(0.002, self.publish_video_stream) # 500 Hz

        URL = "http://192.168.0.103"
        self.vid = cv2.VideoCapture(URL + ":81/stream")


    def publish_video_stream(self):
        ret, frame = self.vid.read()
        if ret:
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher.publish(image_msg)
       

def main(args=None):
    rclpy.init(args=args)
    node = Esp32Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()