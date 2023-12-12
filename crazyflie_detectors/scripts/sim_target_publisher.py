#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from crazyflie_msgs.msg import TargetTransformation
from sensor_msgs.msg import Image
import dt_apriltags as apriltag
import cv2
import numpy as np
from cv_bridge import CvBridge


class ApriltagDetector(Node):
    def __init__(self):
        super().__init__('crazyflie_detectors')
        self.publisher = self.create_publisher(TargetTransformation, 'crazyflie/target_transformation', 10)
        self.camera_sub = self.create_subscription(Image, 'crazyflie/camera', self.camera_callback, 10)
        self.publish_timer = self.create_timer(0.01, self.publish_target)

        self.camera_params = (
            348.62, # fx
            348.62, # fy
            162.0, # cx
            162.0, # cy
        )

        self.detector = apriltag.Detector(
            families='tagStandard41h12',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.5,
            debug=1
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


    def publish_target(self):
        msg = TargetTransformation()
        # concat translation and flattened rotation
        msg.t = self.translation.flatten().tolist() + self.rotation.flatten().tolist()
        self.publisher.publish(msg)
        self.get_logger().info('"%s"' % msg)
    

def main(args=None):
    rclpy.init(args=args)
    target_publisher = ApriltagDetector()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
