#pragma once

#include "crazyflie_localization/my_geometry/matrix_manipulation.h"
#include "crazyflie_localization/my_visual_odometry/visual_odometry.h"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"



class VisualOdometryNode : public rclcpp::Node {

public:
    
    VisualOdometryNode();

private:
    
    void _newImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void _newImuPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void _newImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void _mapCallback();
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_new_image;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _sub_new_imu_pose;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_new_imu;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_feature_image;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub_camera_pose;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_map;
    rclcpp::TimerBase::SharedPtr _timer_map;

    cv::Mat _old_image;
    cv::Matx44d _current_pose;


    const my_vo::VisualOdometry::Ptr _vo;
    


};