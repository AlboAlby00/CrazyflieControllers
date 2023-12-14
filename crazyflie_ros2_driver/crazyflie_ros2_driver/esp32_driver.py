import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_srvs.srv import Empty
import os


class Esp32Driver(Node):

    def __init__(self):
        super().__init__('esp_32_driver')
        self.get_logger().info("esp 32 camera driver running")
        self.publisher = self.create_publisher(Image, 'esp_32/camera', 10)
        self.srv = self.create_service(Empty, '/esp_32/save_frame', self.save_frame_callback)
        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(0.002, self.publish_video_stream)  # 500 Hz
        self.frame = None
        self.n_images_saved = 0
        self.declare_parameter("ip", "192.168.45.169")
        URL = "http://" + self.get_parameter("ip").get_parameter_value().string_value
        self.vid = cv2.VideoCapture(URL + ":81/stream")

    def publish_video_stream(self):
        ret, frame = self.vid.read()
        if ret:
            self.frame = frame
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher.publish(image_msg)

    def save_frame_callback(self, request, response):
        # This service callback can be used to manually trigger frame saving
        package_path = "/home/alboalby00/ros2_galactic_ws/src/deep_learning_in_robotics/crazyflie_ros2_driver"
        image_path = os.path.join(package_path, 'images', f'image_{self.n_images_saved+1}.jpg')
        self.get_logger().info(image_path)
        if self.frame.any():
            self.get_logger().info("Received save request")
            cv2.imwrite(image_path, self.frame)
            self.n_images_saved += 1
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
