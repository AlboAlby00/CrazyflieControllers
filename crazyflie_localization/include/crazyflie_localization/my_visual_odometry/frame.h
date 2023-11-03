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
            static Frame::Ptr createFrame(cv::Mat rgb_img, double time_stamp = -1);

            int _id;            // id of this frame
            double _time_stamp; // when it is recorded

                // -- image features

            std::vector<cv::KeyPoint> _keypoints;
            cv::Mat _descriptors;
            cv::Mat _image;
            cv::Mat get_image_with_keypoints();
            
            
        private:
            static const cv::Ptr<cv::FeatureDetector> _detector;
            

    };

}