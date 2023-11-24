#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "crazyflie_localization/my_datastructures/map_point.h"
#include "crazyflie_localization/my_visual_odometry/frame.h"


namespace my_utils
{
    double calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2);
    double calculate_norm(const cv::Mat &t);
    cv::Point3f transform_point(const cv::Point3f &p3x1, const cv::Mat &T4x4);
    void save_image(const cv::Mat& image);
    void convert_point2f_to_keypoint(const std::vector<cv::Point2f>& points, std::vector<cv::KeyPoint>& keypoints);
    cv::Mat get_descriptors_of_map_points(const std::vector<my_ds::MapPoint::Ptr>& map_points);
    void show_matches(const my_vo::Frame::Ptr frame_1, const my_vo::Frame::Ptr frame_2, const std::vector<cv::DMatch>& matches);

}