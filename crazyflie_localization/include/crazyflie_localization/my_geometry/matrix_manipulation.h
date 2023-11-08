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
    void get_3x4_T_matrix(const cv::Mat& R, const cv::Mat& t, cv::Mat& T);
    void omogeneous_to_3d(const cv::Mat& points_4d, std::list<cv::Point3d>& points_3d);
    cv::Mat get_3x4_identity();
    

}