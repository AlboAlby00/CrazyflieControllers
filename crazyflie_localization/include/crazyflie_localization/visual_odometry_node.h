#pragma once

#include "crazyflie_localization/utils/ORB_tracker.h"
#include "crazyflie_localization/utils/pose_estimator.h"
#include "crazyflie_localization/my_geometry/matrix_manipulation.h"


#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class VisualOdometryNode : public rclcpp::Node {
public:
    VisualOdometryNode();

private:
    void _newImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void _newImuPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_new_image;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_new_imu_pose;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_feature_image;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub_camera_pose;

    cv::Mat _old_image;
    my_vo::ORB_tracker _tracker;
    my_vo::PoseEstimator _poseEstimator;
    cv::Matx44d _current_pose;
    


};