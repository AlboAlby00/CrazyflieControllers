#include "crazyflie_localization/my_visual_odometry/visual_odometry.h"

my_vo::VisualOdometry::VisualOdometry() 
{

}

void my_vo::VisualOdometry::add_new_image(cv::Mat image)
{
    my_vo::Frame::Ptr frame = my_vo::Frame::createFrame(image);
    _frames.push_back(frame);
}

cv::Mat my_vo::VisualOdometry::get_last_image_with_keypoints()
{
    return _frames.back()->get_image_with_keypoints();
}