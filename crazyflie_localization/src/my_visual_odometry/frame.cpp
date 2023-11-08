#include "crazyflie_localization/my_visual_odometry/frame.h"

const cv::Ptr<cv::FeatureDetector> my_vo::Frame::_detector(cv::ORB::create());
int my_vo::Frame::_factory_id = 0;

my_vo::Frame::Ptr my_vo::Frame::createFrame(cv::Mat rgb_img, double time_stamp) 
{
    Frame::Ptr frame(new Frame());
    frame->image = rgb_img;
    _detector->detectAndCompute(rgb_img, cv::noArray(), frame->keypoints, frame->descriptors);

    _factory_id++;
    frame->_id = _factory_id;
    frame->_time_stamp = time_stamp;

    return frame;
}

cv::Mat my_vo::Frame::get_image_with_keypoints()
{   
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(image, keypoints, image_with_keypoints, cv::Scalar(0, 0, 255), 
            cv::DrawMatchesFlags::DEFAULT);
    return image_with_keypoints;
}