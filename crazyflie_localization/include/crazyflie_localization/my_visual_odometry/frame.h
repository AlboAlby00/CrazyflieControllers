#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

namespace my_vo{

    class Frame {
        
        public:

            typedef std::shared_ptr<Frame> Ptr;
            Frame() {}
            ~Frame() {}
            static Frame::Ptr createFrame(cv::Mat rgb_img, cv::Mat K, double time_stamp = -1);

            int _id;            // id of this frame
            double _time_stamp; // when it is recorded
            cv::Mat _K;  // camera intrinsics

            std::vector<cv::KeyPoint> keypoints;
            std::vector<cv::DMatch> matches_2d_3d_with_map;

            cv::Mat descriptors;
            cv::Mat image;
            cv::Mat get_image_with_keypoints();

            cv::Mat _T_world_to_camera;
            
            
        private:
            static const cv::Ptr<cv::FeatureDetector> _detector;
            static int _factory_id;
            

    };

}