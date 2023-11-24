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
    cv::Mat get_3x4_T_matrix(const cv::Mat& T);
    void omogeneous_to_3d(const cv::Mat& points_4d, std::list<cv::Point3d>& points_3d);
    cv::Mat convert_point3f_to_homogeneous(const cv::Point3f& point);
    cv::Mat convert_Rt_to_T(const cv::Mat &R, const cv::Mat &t);
    void convert_T_to_Rt(const cv::Mat &T, cv::Mat &R, cv::Mat &t);
    cv::Mat get_3x4_identity();
    
}