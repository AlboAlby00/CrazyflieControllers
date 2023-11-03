#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "crazyflie_localization/my_visual_odometry/frame.h" 

namespace my_vo{

    class VisualOdometry
    {

        public:
            typedef std::shared_ptr<VisualOdometry> Ptr;
            VisualOdometry();
            bool is_initialized(); 
            cv::Matx44d get_current_pose(); 
            void add_new_image(cv::Mat image); 
            cv::Mat get_last_image_with_keypoints();

        private:
            enum VOState
            {
                BLANK,
                DOING_INITIALIZATION,
                DOING_TRACKING,
                LOST
            };
            VOState _vo_state;

        private:
            // Initialization
            void _estimateMotionAnd3DPoints();
            bool _isVoGoodToInit();
            std::vector<Frame::Ptr> _frames;

    };
}