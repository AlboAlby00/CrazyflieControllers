#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace my_utils
{
    double calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2);

}