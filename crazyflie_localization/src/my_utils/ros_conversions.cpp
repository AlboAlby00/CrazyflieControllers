#include "crazyflie_localization/my_utils/ros_conversions.h"

void my_utils::from_cv_to_ros_pose(const cv::Mat& opencv_pose, geometry_msgs::msg::PoseStamped& ros_pose)
{
    tf2::Vector3 position;  
    tf2::Matrix3x3 rot_robot; 
    for (int i = 0; i < 3; ++i) {
        position[i] = opencv_pose.at<double>(i, 3);
        for (int j = 0; j < 3; ++j) {
            rot_robot[i][j] = opencv_pose.at<double>(i, j);
        }
    }

    tf2::Quaternion orientation;
    rot_robot.getRotation(orientation);

    ros_pose.pose.position.x = position[0];
    ros_pose.pose.position.y = position[1];
    ros_pose.pose.position.z = position[2]; 

    ros_pose.pose.orientation.x = orientation.x();
    ros_pose.pose.orientation.y = orientation.y();  
    ros_pose.pose.orientation.z = orientation.z();  
    ros_pose.pose.orientation.w = orientation.w();

}

void my_utils::create_marker_msg(const my_ds::Map::Ptr map, visualization_msgs::msg::Marker& marker_msg)
{

    marker_msg.ns = "map";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = 0.05; 
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    
    for (const auto& cv_point : map->_map_points) {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = cv_point.second->_pos.x;
        ros_point.y = cv_point.second->_pos.y;
        ros_point.z = cv_point.second->_pos.z;
        marker_msg.points.push_back(ros_point);
        //std::cout << "point added to marker" << std::endl;
    }
}

