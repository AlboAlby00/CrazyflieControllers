#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "crazyflie_localization/my_visual_odometry/frame.h"

namespace my_vo
{

    static const auto detector(cv::ORB::create());
    static const auto matcher(cv::DescriptorMatcher::create("BruteForce-Hamming"));

    void track_features(   
                const Frame::Ptr frame_1, const Frame::Ptr frame_2,
                    std::vector<cv::DMatch>& matches);
    void estimate_pose(   
                const std::vector<cv::KeyPoint> keypoints_1, const std::vector<cv::KeyPoint> keypoints_2, const cv::Mat& K,
                    std::vector<cv::DMatch> matches, cv::Mat &R, cv::Mat &t);
}