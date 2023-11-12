#include "crazyflie_localization/my_datastructures/map.h"

void my_ds::Map::insert_map_point(MapPoint::Ptr map_point)
{
    if (_map_points.find(map_point->_id) == _map_points.end())
    {
        _map_points.insert(make_pair(map_point->_id, map_point));
    }
    else
    {
        _map_points[map_point->_id] = map_point;
    }
}

void my_ds::Map::project_map_points_to_frame(
                const my_vo::Frame::Ptr frame, 
                std::vector<MapPoint::Ptr> &candidate_map_points_in_map, 
                std::vector<cv::Point2f> &candidate_2d_points_in_image,
                cv::Mat &corresponding_map_points_descriptors)
{
    cv::Mat T_camera_to_world = frame->_T_world_to_camera.inv();
    candidate_map_points_in_map.clear();
    corresponding_map_points_descriptors.release();

    for (auto &map_point : _map_points)
    {
        cv::Point3f world_point = map_point.second->_pos;
        cv::Mat descriptor = map_point.second->_descriptor;
        cv::Point3f camera_point = my_utils::transform_point(world_point, T_camera_to_world);
        cv::Point2f pixel = my_geom::cam2pixel(camera_point, frame->_K);

        bool point_is_in_curr_frame = true;
        if(camera_point.z < 0) point_is_in_curr_frame = false;
        const bool is_inside_image = pixel.x > 0 && pixel.y > 0 &&  pixel.x < frame->image.cols && pixel.y < frame->image.rows;
        if (!is_inside_image) point_is_in_curr_frame = false;

        if (point_is_in_curr_frame)
        {
            candidate_map_points_in_map.push_back(map_point.second);
            candidate_2d_points_in_image.push_back(pixel);
            corresponding_map_points_descriptors.push_back(descriptor);
        }
    }
}