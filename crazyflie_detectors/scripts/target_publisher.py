#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crazyflie_msgs.msg import TargetTransformationVector
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
import random
import dt_apriltags as apriltag
import cv2
import numpy as np
from cv_bridge import CvBridge



class TargetPublisher(Node):
    def __init__(self):
        super().__init__('crazyflie_detectors')
        self.publisher = self.create_publisher(TargetTransformationVector, 'crazyflie/target_transformation_vector', 10)
        self.camera_sub = self.create_subscription(Image, 'esp_32/camera', self.camera_callback, 10)

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

        self.x_target = random.uniform(-5, 5)
        self.y_target = random.uniform(-5, 5)
        self.z_target = 0.0

        self.bridge = CvBridge()
        # create gps subscriber
#        self.gps_sub = self.create_subscription(PointStamped, 'crazyflie/gps', self.gps_callback, 10)
    
    
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
            print('hi')
        else:
            tag_pos = [[0.0], [0.0], [0.0]]

        msg = TargetTransformationVector()
        msg.x = tag_pos[0][0]
        msg.y = tag_pos[1][0]
        msg.z = tag_pos[2][0]
        self.publisher.publish(msg)
        self.get_logger().info('Publishing here: "%s"' % msg)

    
    def gps_callback(self, msg):
        self.gps_x = msg.point.x
        self.gps_y = msg.point.y
        self.gps_z = msg.point.z

        # publish target vector
        vec_to_target = TargetTransformationVector()
        vec_to_target.x = self.x_target - self.gps_x
        vec_to_target.y = self.y_target - self.gps_y
        vec_to_target.z = self.z_target - self.gps_z

        self.publisher.publish(vec_to_target)
        # log vec_to_target
        self.get_logger().info('Publishing: "%s"' % vec_to_target)


def main(args=None):
    rclpy.init(args=args)
    target_publisher = TargetPublisher()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
