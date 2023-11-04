#include "crazyflie_localization/my_visual_odometry/visual_odometry.h"

my_vo::VisualOdometry::VisualOdometry() 
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

    if(_vo_state == BLANK)
    {
        my_vo::Frame::Ptr frame = my_vo::Frame::createFrame(image);
        _frames.push_back(frame);
        _vo_state = DOING_INITIALIZATION;
    } 
    else if (_vo_state == DOING_INITIALIZATION)
    {
        my_vo::Frame::Ptr past_frame = _frames.back();
        my_vo::Frame::Ptr current_frame = my_vo::Frame::createFrame(image);
        _frames.push_back(current_frame);

        std::vector<cv::DMatch> matches;
        my_vo::track_features(past_frame, current_frame, matches);

        if(matches.size() < MIN_FEATURES_TO_START_TRACKING)
        {
            return;
        } 
        else 
        {
            cv::Mat R,t;
            my_vo::estimate_pose(current_frame->keypoints,past_frame->keypoints,_K,matches,R,t);
            
            cv::Mat pnts3D(4,current_frame->keypoints.size(),CV_64F);
            //cv::triangulatePoints( _K, _K, current_frame->keypoints, past_frame->keypoints, pnts3D);

            _vo_state = DOING_TRACKING;
        }
    }
    else if (_vo_state == DOING_TRACKING)
    {
        /* code */
    }
    
}

cv::Mat my_vo::VisualOdometry::get_last_image_with_keypoints()
{
    return _frames.back()->get_image_with_keypoints();
}