#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace my_vo{

    class ORB_tracker {
        
        public:
            ORB_tracker();
            void track_features(   
                const cv::Mat& image_1, const cv::Mat& image_2,
                std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2,
                std::vector<cv::DMatch>& matches); 
        
        private:
            const cv::Ptr<cv::FeatureDetector> detector;
            const cv::Ptr<cv::DescriptorMatcher> matcher;

    };

}