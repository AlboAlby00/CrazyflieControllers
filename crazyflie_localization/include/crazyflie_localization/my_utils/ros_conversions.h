#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "crazyflie_localization/my_datastructures/map.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"




namespace my_utils{

    void from_cv_to_ros_pose(const cv::Mat& T, geometry_msgs::msg::PoseStamped& pose);
    void create_marker_msg(const my_ds::Map::Ptr map, visualization_msgs::msg::Marker& marker_msg);

}