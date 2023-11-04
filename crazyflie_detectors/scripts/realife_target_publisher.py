#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from crazyflie_msgs.msg import TargetTransformationVector
from geometry_msgs.msg import PointStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
import random
import dt_apriltags as apriltag
import cv2
import numpy as np
from cv_bridge import CvBridge


class ApriltagDetector(Node):
    def __init__(self):
        super().__init__('crazyflie_detectors')
        self.publisher = self.create_publisher(TargetTransformationVector, 'crazyflie/target_transformation_vector', 10)
        self.camera_sub = self.create_subscription(Image, 'esp_32/camera', self.camera_callback, 10)
        self.publish_timer = self.create_timer(0.01, self.publish_target)

        self.camera_params = (
            301.01404911, # fx
            308.08275162, # fy
            161.03012367, # cx
            139.61657559, # cy
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

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

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
            tag_size=0.19
        )
        if len(detections) > 0:
            tag_pos = detections[0].pose_t
        else:
            tag_pos = [[0.0], [0.0], [0.0]]

        self.target_x = tag_pos[0][0]
        self.target_y = tag_pos[1][0]
        self.target_z = tag_pos[2][0]

    def publish_target(self):
        msg = TargetTransformationVector()
        msg.x = self.target_x
        msg.y = self.target_y
        msg.z = self.target_z
        self.publisher.publish(msg)
        self.get_logger().info('Publishing here: "%s"' % msg)
    

def main(args=None):
    rclpy.init(args=args)
    target_publisher = ApriltagDetector()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
