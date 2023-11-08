#include "crazyflie_localization/my_visual_odometry/visual_odometry.h"

my_vo::VisualOdometry::VisualOdometry() : _vo_state(VOState::BLANK)
{

}

void my_vo::VisualOdometry::set_intrinsics(double fx, double fy, double cx, double cy)
{
    _K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0,  fy, cy, 0,  0, 1);
    std::cout << fy << std::endl;
}

void my_vo::VisualOdometry::add_new_image(cv::Mat image)
{
    // to be changed later
    my_vo::Frame::Ptr frame = my_vo::Frame::createFrame(image);
    _frames.push_back(frame);
    if(_frames.size() > 200)
    {
        _frames.pop_front();
    }

    if(_vo_state == VOState::BLANK)
    {
        std::cout << "blank" << std::endl;
        my_vo::Frame::Ptr frame = my_vo::Frame::createFrame(image);
        _frames.push_back(frame);
        _reference_keyframe = frame;
        _vo_state =  VOState::DOING_INITIALIZATION;
    } 
    else if (_vo_state ==  VOState::DOING_INITIALIZATION)
    {
        std::cout << "doing initialization" << std::endl;
        my_vo::Frame::Ptr current_frame = my_vo::Frame::createFrame(image);
        _frames.push_back(current_frame);

        std::vector<cv::DMatch> matches;
        my_vo::track_features(_reference_keyframe, current_frame, matches);

        if(matches.size() < MIN_FEATURES_TO_START_TRACKING)
        {
            return;
        } 

        std::cout << "creating first 3d points" << std::endl;

        cv::Mat R,t;
        std::cout << "a0" << std::endl;
        my_vo::estimate_pose(_reference_keyframe->keypoints,current_frame->keypoints,_K,matches,R,t);

        std::cout << "a1" << std::endl;
        
        cv::Mat pnts3D(4,current_frame->keypoints.size(),CV_64F);

        cv::Mat T1, T2;
        my_geom::get_3x4_T_matrix(cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F), T1);
        my_geom::get_3x4_T_matrix(R, t, T2);

        bool is_ready = _is_ready_for_initialization(_reference_keyframe->keypoints,current_frame->keypoints,_K,matches);
        if( ! is_ready)
        {
            return;
        }

        std::cout << "ready for initialization!" << std::endl;

        std::vector<cv::Point2f> points_1, points_2;
        my_vo::convert_keypoints_to_point2f( _reference_keyframe->keypoints, current_frame->keypoints, _K,
                matches, points_1, points_2);


        std::cout << "b" << std::endl;

        cv::Mat points_4d;
        cv::triangulatePoints( T1, T2, points_1, points_2, points_4d );

        std::cout << "c" << std::endl;
        
        // Transform omogeneus coordinates to 3D points
        for ( int i=0; i<points_4d.cols; i++ )
        {
            cv::Mat x = points_4d.col(i);
            x /= x.at<float>(3,0); 
            cv::Point3d p (
                x.at<float>(0,0), 
                x.at<float>(1,0), 
                x.at<float>(2,0) 
            );
            map.push_back( p );
        }

        std::cout << " T is " << T2 << std::endl;
        
        std::cout << " size of map: " << map.size() << std::endl;

        _vo_state =  VOState::DOING_TRACKING;

    }
    else if (_vo_state ==  VOState::DOING_TRACKING)
    {
        /* code */
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
    return _frames.back()->get_image_with_keypoints();
}