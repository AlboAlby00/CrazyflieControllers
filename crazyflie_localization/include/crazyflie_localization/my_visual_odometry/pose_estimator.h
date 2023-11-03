#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace my_vo{

    class  PoseEstimator{
        
        public:
            PoseEstimator(cv::Mat K);
            void estimate(   std::vector<cv::KeyPoint> keypoints_1, std::vector<cv::KeyPoint> keypoints_2,
                std::vector<cv::DMatch> matches, cv::Mat &R, cv::Mat &t); 
        
        private:
            const cv::Mat _K;

    };

}