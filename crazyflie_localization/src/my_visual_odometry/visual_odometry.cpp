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
        _vo_state =  VOState::DOING_INITIALIZATION;
    } 
    else if (_vo_state ==  VOState::DOING_INITIALIZATION)
    {
        std::cout << "doing initialization" << std::endl;
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
            
            std::cout << "creating first 3d points" << std::endl;

            cv::Mat R,t;
            std::cout << "a0" << std::endl;
            my_vo::estimate_pose(current_frame->keypoints,past_frame->keypoints,_K,matches,R,t);

            std::cout << "a1" << std::endl;
            
            cv::Mat pnts3D(4,current_frame->keypoints.size(),CV_64F);
            cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
                                        1, 0, 0, 0,
                                        0, 1, 0, 0,
                                        0, 0, 1, 0);
            cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
                            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

            std::cout << "a2" << std::endl;

            std::vector<cv::Point2f> pts_1, pts_2;
            for ( cv::DMatch m:matches )
            {
                pts_1.push_back( my_geom::pixel2CamNormPlane( current_frame->keypoints[m.queryIdx].pt, _K) );
                pts_2.push_back( my_geom::pixel2CamNormPlane( past_frame->keypoints[m.trainIdx].pt, _K) );
            }

            std::cout << "b" << std::endl;

            cv::Mat pts_4d;
            cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );

            std::cout << "c" << std::endl;
            
            // Transform omogeneus coordinates to 3D points
            for ( int i=0; i<pts_4d.cols; i++ )
            {
                cv::Mat x = pts_4d.col(i);
                x /= x.at<float>(3,0); 
                cv::Point3d p (
                    x.at<float>(0,0), 
                    x.at<float>(1,0), 
                    x.at<float>(2,0) 
                );
                map.push_back( p );
            }

            std::cout << "d" << std::endl;
            
            std::cout << " size of map: " << map.size() << std::endl;

            _vo_state =  VOState::DOING_TRACKING;
        }
    }
    else if (_vo_state ==  VOState::DOING_TRACKING)
    {
        /* code */
    }
    
}

cv::Mat my_vo::VisualOdometry::get_last_image_with_keypoints()
{
    return _frames.back()->get_image_with_keypoints();
}