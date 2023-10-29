#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


class VisualOdometryNode : public rclcpp::Node {
public:
    VisualOdometryNode();

private:
    void _newImageCallback(const sensor_msgs::msg::Image::SharedPtr command);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_new_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_feature_image;
    cv::Mat  _old_image;


};