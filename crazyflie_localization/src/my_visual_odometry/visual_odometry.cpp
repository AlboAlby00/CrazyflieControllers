#include "crazyflie_localization/my_visual_odometry/visual_odometry.h"

my_vo::VisualOdometry::VisualOdometry() : _vo_state(VOState::BLANK)
{
    map = std::make_shared<my_ds::Map>();
}

void my_vo::VisualOdometry::set_intrinsics(double fx, double fy, double cx, double cy)
{
    _K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0,  fy, cy, 0,  0, 1);
    std::cout << fy << std::endl;
}

void my_vo::VisualOdometry::add_frame_to_buffer(Frame::Ptr frame)
{
    if (_buffer_frames.size() > 100)
        _buffer_frames.pop();
    _buffer_frames.push(frame);

}

bool my_vo::VisualOdometry::is_initialized()
{
    return _vo_state != VOState::BLANK && _vo_state != VOState::DOING_INITIALIZATION;
}

cv::Mat my_vo::VisualOdometry::get_current_pose()
{
    cv::Mat identity = cv::Mat::eye(4,4,CV_64F);
    cv::Mat pose = is_initialized() ? _buffer_frames.back()->_T_world_to_camera : identity;

    return pose;
}

void my_vo::VisualOdometry::add_new_image(cv::Mat image)
{


    if(_vo_state == VOState::BLANK)
    {
        my_vo::Frame::Ptr frame = my_vo::Frame::createFrame(image, _K);
        frame->_T_world_to_camera = cv::Mat::eye(4,4,CV_64F);

        _reference_keyframe = frame;
        _keyframes.insert(std::make_pair(frame->_id, frame));

        add_frame_to_buffer(frame);

        my_utils::save_image(frame->get_image_with_keypoints());

        _vo_state =  VOState::DOING_INITIALIZATION;

    } 
    else if (_vo_state ==  VOState::DOING_INITIALIZATION)
    {
        my_vo::Frame::Ptr current_frame = my_vo::Frame::createFrame(image, _K);
        add_frame_to_buffer(current_frame);

        std::vector<cv::DMatch> matches;
        my_vo::track_features(_reference_keyframe, current_frame, matches);

        cv::Mat R,t;
        my_vo::estimate_pose(_reference_keyframe->keypoints,current_frame->keypoints,_K,matches,R,t);
        
        cv::Mat T1_3x4, T2_3x4;
        my_geom::get_3x4_T_matrix(cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F), T1_3x4);
        my_geom::get_3x4_T_matrix(R, t, T2_3x4);

        bool is_ready = _is_ready_for_initialization(_reference_keyframe->keypoints,current_frame->keypoints,_K,matches);
        if( ! is_ready)
        {
            return;
        }

        my_utils::save_image(current_frame->get_image_with_keypoints());
        my_utils::show_matches(_reference_keyframe, current_frame, matches);

        std::cout << " t is: " << t << std::endl;

        std::cout << "ready for initialization!" << std::endl;

        std::vector<cv::Point2f> points_1, points_2;
        my_vo::convert_keypoints_to_point2f( _reference_keyframe->keypoints, current_frame->keypoints, _K,
                matches, points_1, points_2);
        
        cv::Mat points_4d;

        // triangulate and add points to map
        cv::triangulatePoints( T1_3x4, T2_3x4, points_1, points_2, points_4d );


        std::list<cv::Point3d> points_3d;
        my_geom::omogeneous_to_3d(points_4d, points_3d);
        cv::Mat points_3d_descriptor;
        my_vo::get_descriptor_of_points_3d(current_frame->descriptors, matches ,points_3d_descriptor);
        _add_points_to_map(points_3d, points_3d_descriptor);


        cv::Mat T2_4x4 = my_geom::convert_Rt_to_T(R, t);
        current_frame->_T_world_to_camera = T2_4x4.inv();


        _vo_state =  VOState::DOING_TRACKING;

    }
    else if (_vo_state ==  VOState::DOING_TRACKING)
    {

        my_vo::Frame::Ptr current_frame = my_vo::Frame::createFrame(image, _K);
        current_frame->_T_world_to_camera = cv::Mat::eye(4,4,CV_64F);
        add_frame_to_buffer(current_frame);


        // map 2d keypoints to map points
        std::vector<cv::Point2f> points_2d;
        std::vector<cv::Point3f> points_3d;
        my_vo::find_2d_3d_correspondences(map, current_frame, points_2d, points_3d);

        //std::cout << "points_2d.size(): " << points_2d.size() << std::endl;
        //std::cout << "points_3d.size(): " << points_3d.size() << std::endl;

        // estimate rotation and translation using PnP

        cv::Mat rvec(3,1,CV_64F);
        cv::Mat tvec(3,1,CV_64F);

        cv::Mat distCoeffs(4,1,CV_64F);
        distCoeffs.at<double>(0) = 0;
        distCoeffs.at<double>(1) = 0;
        distCoeffs.at<double>(2) = 0;
        distCoeffs.at<double>(3) = 0;

        _vo_state = VOState::LOST;

        cv::solvePnPRansac(points_3d, points_2d, _K, distCoeffs, rvec, tvec);
 
        std::cout << "rvec: " << rvec << std::endl;
        std::cout << "tvec: " << tvec << std::endl;

        // triangulate new points


    }
    
}

void my_vo::VisualOdometry::_add_points_to_map(const std::list<cv::Point3d>& points_3d, const cv::Mat& points_3d_descriptors)
{
    auto point_3d_iterator = points_3d.begin();
    for (int i=0; i < points_3d.size(); i++)
    {
        my_ds::MapPoint::Ptr map_point = my_ds::MapPoint::createMapPoint(*point_3d_iterator,points_3d_descriptors.row(i));
        map->insert_map_point(map_point);
        ++point_3d_iterator;
    }

}

bool my_vo::VisualOdometry::_is_ready_for_initialization(
    const std::vector<cv::KeyPoint>& keypoints_1 , const std::vector<cv::KeyPoint>& keypoints_2 , 
                    const cv::Mat K, const std::vector<cv::DMatch>& matches)
{
    bool enough_correspondence, large_baseline;
    
    enough_correspondence = matches.size() > MIN_FEATURES_TO_START_TRACKING;

    double pixel_distance = compute_keypoints_mean_distance(keypoints_1, keypoints_2, matches);
    large_baseline = pixel_distance > MIN_FEATURES_PIXEL_DISTANCE;

    return enough_correspondence && large_baseline;
}

cv::Mat my_vo::VisualOdometry::get_last_image_with_keypoints()
{
    return _buffer_frames.back()->get_image_with_keypoints();
}