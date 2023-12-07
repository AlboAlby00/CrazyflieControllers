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
#include "crazyflie_localization/my_datastructures/map_point.h"
#include "crazyflie_localization/my_datastructures/map.h"


namespace my_vo{

    class VisualOdometry
    {   
        public:
            my_ds::Map::Ptr map;

        public:
            typedef std::shared_ptr<VisualOdometry> Ptr;
            VisualOdometry();
            void add_frame_to_buffer(Frame::Ptr frame);
            bool is_initialized(); 
            cv::Mat get_current_pose(); 
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
            const int MIN_FEATURES_TO_START_TRACKING = 30;
            const int MIN_FEATURES_PIXEL_DISTANCE = 100;
            const double MIN_DISTANCE_BETWEEN_KEYFRAMES = 0.2;

        private:
            // Initialization
            bool _is_ready_for_initialization(
                const std::vector<cv::KeyPoint>& keypoints_1 , const std::vector<cv::KeyPoint>& keypoints_2 , 
                    const cv::Mat K, const std::vector<cv::DMatch>& matches);
            bool _distance_between_frames_is_enough(const Frame::Ptr frame_1, const Frame::Ptr frame_2);
            cv::Mat _K;
            void _add_points_to_map(const std::list<cv::Point3d>& points_3d, const cv::Mat& points_3d_descriptors);
            void _add_keyframe(const Frame::Ptr frame);

            std::queue<Frame::Ptr> _buffer_frames;

            Frame::Ptr _reference_keyframe;
            

    };
}