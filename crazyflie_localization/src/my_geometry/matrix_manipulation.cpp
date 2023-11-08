#include "crazyflie_localization/my_geometry/matrix_manipulation.h"

void my_geom::get_T_matrix(const cv::Matx33d& R, const cv::Vec3d& t, cv::Matx44d& T )
{
    T.get_minor<3, 3>(0, 0) = R;
    T.get_minor<3, 1>(0, 3) = t;
}

void my_geom::get_T_matrix(
    const geometry_msgs::msg::PoseStamped::SharedPtr& pose, cv::Matx44d& T)
{
    
    // Extract the translation (x, y, z)
    cv::Vec3d t(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
    cv::Vec4d quaternion(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);

}

void my_geom::get_3x4_T_matrix(const cv::Mat& R, const cv::Mat& t, cv::Mat& T)
{
    T = (cv::Mat_<float>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
}


