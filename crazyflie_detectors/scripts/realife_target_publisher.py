#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import dt_apriltags as apriltag
import cv2
import numpy as np
from cv_bridge import CvBridge


class ApriltagDetector(Node):
    def __init__(self):
        super().__init__('crazyflie_detectors')
        self.translation_publisher = self.create_publisher(PointStamped, 'crazyflie/at_translation', 10)
        self.rotation_publisher = self.create_publisher(PoseStamped, 'crazyflie/at_rotation', 10)
        self.camera_sub = self.create_subscription(Image, 'esp_32/camera', self.camera_callback, 10)
        self.publish_translation_timer = self.create_timer(0.01, self.publish_translation)
        # self.publish_rotation_timer = self.create_timer(0.01, self.publish_rotation)

        self.camera_params = (
            594.48, # fx
            594.56, # fy
            341.62, # cx
            239.50, # cy
        )

        self.detector = apriltag.Detector(
            families='tagStandard41h12',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.translation = np.zeros(3)
        self.rotation = np.eye(3)

        self.bridge = CvBridge()
    
    
    def camera_callback(self, image_msg):
        image = image_msg.data
        # convert from ros image to cv2 image
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        # convert to grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            image, 
            estimate_tag_pose=True, 
            camera_params=self.camera_params, 
            tag_size=0.1
        )
        if len(detections) > 0:
            tag_pos = detections[0].pose_t
            rotation = detections[0].pose_R
        else:
            tag_pos = [[0.0], [0.0], [0.0]]
            rotation = np.eye(3)

        self.translation = np.array([
            tag_pos[0][0],
            tag_pos[1][0],
            tag_pos[2][0]
        ])
        self.rotation = rotation


    def publish_translation(self):
        msg = PointStamped()
        msg.point.x = self.translation[0]
        msg.point.y = self.translation[1]
        msg.point.z = self.translation[2]
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)
        self.get_logger().info('Publishing translation: "%s"' % msg)
    
    def publish_rotation(self):
        msg = PoseStamped()
        msg.pose.position.x = self.translation[0]
        msg.pose.position.y = self.translation[1]
        msg.pose.position.z = self.translation[2]
        msg.pose.orientation.x = self.rotation[0][0]
        msg.pose.orientation.y = self.rotation[1][0]
        msg.pose.orientation.z = self.rotation[2][0]
        msg.pose.orientation.w = self.rotation[0][1]
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)
        self.get_logger().info('Publishing rotation: "%s"' % msg)
    

def main(args=None):
    rclpy.init(args=args)
    target_publisher = ApriltagDetector()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
