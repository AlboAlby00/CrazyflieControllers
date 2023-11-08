#include "crazyflie_localization/my_visual_odometry/visual_odometry.h"

my_vo::VisualOdometry::VisualOdometry() : _vo_state(VOState::BLANK)
{

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

cv::Matx34d my_vo::VisualOdometry::get_current_pose()
{
    cv::Mat identity = my_geom::get_3x4_identity();
    cv::Mat pose = is_initialized() ? _buffer_frames.back()->_T_world_to_camera : identity;

    return pose;
}

void my_vo::VisualOdometry::add_new_image(cv::Mat image)
{


    if(_vo_state == VOState::BLANK)
    {
        my_vo::Frame::Ptr frame = my_vo::Frame::createFrame(image);
        frame->_T_world_to_camera = my_geom::get_3x4_identity();

        _reference_keyframe = frame;
        _keyframes.insert(std::make_pair(frame->_id, frame));

        add_frame_to_buffer(frame);

        my_utils::save_image(frame->get_image_with_keypoints());

        _vo_state =  VOState::DOING_INITIALIZATION;

    } 
    else if (_vo_state ==  VOState::DOING_INITIALIZATION)
    {
        my_vo::Frame::Ptr current_frame = my_vo::Frame::createFrame(image);
        add_frame_to_buffer(current_frame);

        std::vector<cv::DMatch> matches;
        my_vo::track_features(_reference_keyframe, current_frame, matches);

        cv::Mat R,t;
        my_vo::estimate_pose(_reference_keyframe->keypoints,current_frame->keypoints,_K,matches,R,t);
        
        cv::Mat T1, T2;
        my_geom::get_3x4_T_matrix(cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F), T1);
        my_geom::get_3x4_T_matrix(R, t, T2);

        bool is_ready = _is_ready_for_initialization(_reference_keyframe->keypoints,current_frame->keypoints,_K,matches);
        if( ! is_ready)
        {
            return;
        }

        my_utils::save_image(current_frame->get_image_with_keypoints());
        std::cout << " t is: " << t << std::endl;

        std::cout << "ready for initialization!" << std::endl;

        std::vector<cv::Point2f> points_1, points_2;
        my_vo::convert_keypoints_to_point2f( _reference_keyframe->keypoints, current_frame->keypoints, _K,
                matches, points_1, points_2);


        cv::Mat points_4d;

        cv::triangulatePoints( T1, T2, points_1, points_2, points_4d );

        my_geom::omogeneous_to_3d(points_4d, map);

        current_frame->_T_world_to_camera = T2;

        
        std::cout << " size of map: " << map.size() << std::endl;

        _vo_state =  VOState::DOING_TRACKING;

    }
    else if (_vo_state ==  VOState::DOING_TRACKING)
    {
        my_vo::Frame::Ptr current_frame = my_vo::Frame::createFrame(image);
        current_frame->_T_world_to_camera = my_geom::get_3x4_identity();
        add_frame_to_buffer(current_frame);
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