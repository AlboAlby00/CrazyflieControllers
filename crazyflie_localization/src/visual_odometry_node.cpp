#include "crazyflie_localization/visual_odometry_node.h"


VisualOdometryNode::VisualOdometryNode() : Node("visual_odometry_node"),
     _vo(new my_vo::VisualOdometry)
{
    this->declare_parameter("fx",0.0); double fx = this->get_parameter("fx").as_double();
    this->declare_parameter("fy",0.0); double fy = this->get_parameter("fy").as_double();
    this->declare_parameter("cx",0.0); double cx = this->get_parameter("cx").as_double();
    this->declare_parameter("cy",0.0); double cy = this->get_parameter("cy").as_double();

    _vo->set_intrinsics(fx, fy, cx, cy);

    _sub_new_image =  this->create_subscription<sensor_msgs::msg::Image>(
            "/crazyflie/camera", 10,
            std::bind(&VisualOdometryNode::_newImageCallback, this, std::placeholders::_1));

    _sub_new_imu_pose = this->create_subscription<geometry_msgs::msg::Point>(
            "/crazyflie/imu_pose", 10,
            std::bind(&VisualOdometryNode::_newImuPoseCallback, this, std::placeholders::_1));

    _sub_new_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/crazyflie/imu", 10,
            std::bind(&VisualOdometryNode::_newImuCallback, this, std::placeholders::_1));

    _pub_feature_image = this->create_publisher<sensor_msgs::msg::Image>(
            "/crazyflie/camera_features", 10);

    _pub_camera_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/crazyflie/camera_pose", 10);

    _pub_map = this->create_publisher<visualization_msgs::msg::Marker>(
            "/crazyflie/map", 10);

    _timer_pub = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&VisualOdometryNode::_pubCallback, this));
    
}

void VisualOdometryNode::_newImuPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "imu pose x is %f", msg->x);
}

void VisualOdometryNode::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "imu msg received at %d", msg->header.stamp.sec);
}

void VisualOdometryNode::_newImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "image received");
    cv::Mat new_image = cv_bridge::toCvCopy(msg)->image;
    assert(new_image.data != nullptr);
    if(!new_image.empty()) _vo->add_new_image(new_image);
    cv::Mat new_image_with_keypoints = _vo->get_last_image_with_keypoints();

     // Convert the OpenCV image back to a sensor_msgs::msg::Image
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->image = new_image_with_keypoints;
    sensor_msgs::msg::Image msg_image_with_keypoints = *cv_ptr->toImageMsg();

    msg_image_with_keypoints.header.stamp = get_clock()->now();
    msg_image_with_keypoints.header.frame_id = "camera";
    msg_image_with_keypoints.encoding = msg->encoding;
    _pub_feature_image->publish(msg_image_with_keypoints);

}

void VisualOdometryNode::_pubCallback()
{
    // publish map

    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "map";
    marker_msg.header.stamp = now();
    my_utils::create_marker_msg(_vo->map, marker_msg);

    _pub_map->publish(marker_msg);

    // publish last frame pose

    cv::Mat last_frame_opencv_pose = _vo->get_current_pose();

    geometry_msgs::msg::PoseStamped pose_ros_msg;
    pose_ros_msg.header.frame_id = "base_link";
    pose_ros_msg.header.stamp = now();
    my_utils::from_cv_to_ros_pose(last_frame_opencv_pose, pose_ros_msg);

    _pub_camera_pose->publish(pose_ros_msg);

}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
