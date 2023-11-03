#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


class VisualOdometry
{
    public:
        VisualOdometry();
        bool is_initialized(); 
        cv::Matx44d get_current_pose(); 
        void add_new_image(cv::Mat image);                

    private:
        enum VOState
        {
            BLANK,
            DOING_INITIALIZATION,
            DOING_TRACKING,
            LOST
        };
        VOState vo_state_;

    private:
        // Initialization
        void _estimateMotionAnd3DPoints();
        bool _isVoGoodToInit();




};