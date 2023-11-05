#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace my_geom
{
    void get_T_matrix(const geometry_msgs::msg::PoseStamped::SharedPtr& pose, cv::Matx44d& T);
    void get_T_matrix(const cv::Matx33d& R, const cv::Vec3d& t, cv::Matx44d& T );
    void convert_keypoints_to_point2f(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points );
    

}