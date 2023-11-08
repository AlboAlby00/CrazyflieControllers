#pragma once

#include "crazyflie_localization/my_geometry/matrix_manipulation.h"
#include "crazyflie_localization/my_visual_odometry/visual_odometry.h"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"



class VisualOdometryNode : public rclcpp::Node {

public:
    
    VisualOdometryNode();

private:
    
    void _newImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void _newImuPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void _newImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void _publishLastFramePoseCallback();
    void _pubCallback();
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_new_image;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _sub_new_imu_pose;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_new_imu;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_feature_image;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub_camera_pose;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_map;
    rclcpp::TimerBase::SharedPtr _timer_pub;

    cv::Mat _old_image;

    const my_vo::VisualOdometry::Ptr _vo;
    


};