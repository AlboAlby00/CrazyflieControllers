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
/*
    // Convert quaternion to a rotation matrix
    cv::Matx33d R;
    cv::Rodrigues(quaternion.toRotationMatrix(), R);

    // Create a 4x4 transformation matrix
    T.get_minor<3, 3>(0, 0) = R;
    T.get_minor<3, 1>(0, 3) = t;

    pose->pose.orientation.
    */


}

void my_geom::convert_keypoints_to_point2f(
    const std::vector<cv::KeyPoint>& keypoints , std::vector<cv::Point2f>& points )
{

}
