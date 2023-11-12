#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "crazyflie_localization/my_datastructures/map_point.h"
#include "crazyflie_localization/my_visual_odometry/frame.h"
#include "crazyflie_localization/my_geometry/camera.h"
#include "crazyflie_localization/my_geometry/matrix_manipulation.h"
#include "crazyflie_localization/my_utils/opencv_functions.h"

namespace my_ds
{

    class Map
    {
        public:
            typedef std::shared_ptr<Map> Ptr;

            Map() {}
            std::unordered_map<int, MapPoint::Ptr> _map_points;

            void insert_map_point(MapPoint::Ptr map_point);
            void project_map_points_to_frame(
                const my_vo::Frame::Ptr frame, 
                std::vector<MapPoint::Ptr> &candidate_map_points_in_map, 
                std::vector<cv::Point2f>  &candidate_2d_points_in_image,
                cv::Mat &corresponding_map_points_descriptors);
    };
}