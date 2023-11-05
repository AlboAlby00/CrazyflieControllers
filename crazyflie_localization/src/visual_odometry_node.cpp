#include "crazyflie_localization/visual_odometry_node.h"


VisualOdometryNode::VisualOdometryNode() : Node("visual_odometry_node"),
     _current_pose(cv::Matx44d::eye()), _vo(new my_vo::VisualOdometry)
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

    _timer_map = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&VisualOdometryNode::_mapCallback, this));
    
}

void VisualOdometryNode::_newImuPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    //msg->pose.orientation
    //msg->pose.position
}

void VisualOdometryNode::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    //msg->pose.orientation
    //msg->pose.position
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

    

    
    /*
    // first image arrives
    if(_old_image.data == nullptr ) 
    {
        _old_image = new_image;
        return;
    }

    // select and match features
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    _tracker.track_features(_old_image, new_image, keypoints_1, keypoints_2, matches);
    const std::size_t number_of_matches = matches.size();
    RCLCPP_INFO(this->get_logger(), "number of good matches is %ld", number_of_matches);

    if(number_of_matches < 8)
    {
        return;
    }

    // estimate R, t
    cv::Mat R,t;
    _poseEstimator.estimate(keypoints_1,keypoints_2,matches,R,t);

    // calculate next pose
    cv::Matx44d T;
    my_geom::get_T_matrix(R,t,T);
    _current_pose = _current_pose * T.inv();
    


    // Convert the rotation to a quaternion
    cv::Matx33d rotation_matrix;
    rotation_matrix = _current_pose.get_minor<3,3>(0,0);
    cv::Matx33d identity_matrix = cv::Matx33d::eye();
    cv::Matx33d rotation_difference = rotation_matrix - identity_matrix;
    cv::Mat rotation_vector;
    cv::Rodrigues(rotation_difference, rotation_vector);

    // publish R,t
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header.stamp = get_clock()->now();
    camera_pose.header.frame_id = "camera";

    // Set the position using the translation
    camera_pose.pose.position.x = t.at<double>(0, 0);
    camera_pose.pose.position.y = t.at<double>(1, 0);
    camera_pose.pose.position.z = t.at<double>(2, 0);

    // Set the orientation using the rotation vector
    camera_pose.pose.orientation.x = rotation_vector.at<double>(0, 0);
    camera_pose.pose.orientation.y = rotation_vector.at<double>(1, 0);
    camera_pose.pose.orientation.z = rotation_vector.at<double>(2, 0);
    camera_pose.pose.orientation.w = 1.0;

    _pub_camera_pose->publish(camera_pose);

    cv::Mat new_image_with_keypoints;
    cv::drawKeypoints(new_image, keypoints_1, new_image_with_keypoints, cv::Scalar(0, 0, 255), 
            cv::DrawMatchesFlags::DEFAULT);
    
    // Convert the OpenCV image back to a sensor_msgs::msg::Image
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->image = new_image_with_keypoints;
    sensor_msgs::msg::Image image_with_features = *cv_ptr->toImageMsg();


    image_with_features.header.stamp = get_clock()->now();
    image_with_features.header.frame_id = "camera";
    image_with_features.encoding = msg->encoding;
    _pub_feature_image->publish(image_with_features);

    _old_image = new_image;

    */
}

void VisualOdometryNode::_mapCallback()
{
    // publish map

    // Create a Marker message
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "map"; // Set your desired frame
    marker_msg.header.stamp = now();
    marker_msg.ns = "my_namespace";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = 0.1; // Set your desired scale
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    
    // Convert cv::Point3d to geometry_msgs::msg::Point
    for (const auto& cv_point : _vo->map ) {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = cv_point.x;
        ros_point.y = cv_point.y;
        ros_point.z = cv_point.z;
        marker_msg.points.push_back(ros_point);
    }

    _pub_map->publish(marker_msg);
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
