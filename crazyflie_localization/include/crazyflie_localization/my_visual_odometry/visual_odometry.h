#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <list>
#include <stdio.h>

#include "crazyflie_localization/my_geometry/camera.h"
#include "crazyflie_localization/my_geometry/matrix_manipulation.h"
#include "crazyflie_localization/my_visual_odometry/frame.h" 
#include "crazyflie_localization/my_visual_odometry/matching_and_tracking.h"


namespace my_vo{

    class VisualOdometry
    {   
        public:
            std::list<cv::Point3d> map;

        public:
            typedef std::shared_ptr<VisualOdometry> Ptr;
            VisualOdometry();
            bool is_initialized(); 
            cv::Matx44d get_current_pose(); 
            void add_new_image(cv::Mat image); 
            void set_intrinsics(double fx, double fy, double cx, double cy);
            cv::Mat get_last_image_with_keypoints();

        private:
            enum class VOState
            {
                BLANK,
                DOING_INITIALIZATION,
                DOING_TRACKING,
                LOST
            };
            VOState _vo_state;
            const int MIN_FEATURES_TO_START_TRACKING = 15;
            const int MIN_FEATURES_PIXEL_DISTANCE = 50;

        private:
            // Initialization
            bool _is_ready_for_initialization(
                const std::vector<cv::KeyPoint>& keypoints_1 , const std::vector<cv::KeyPoint>& keypoints_2 , 
                    const cv::Mat K, const std::vector<cv::DMatch>& matches);
            cv::Mat _K;

            std::list<Frame::Ptr> _frames;
            Frame::Ptr _reference_keyframe;
            

    };
}