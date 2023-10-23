#include "crazyflie_localization/visual_odometry_node.h"


VisualOdometryNode::VisualOdometryNode() : Node("visual_odometry_node")
{
    _sub_new_image =  this->create_subscription<sensor_msgs::msg::Image>(
            "/crazyflie/camera",
            10,
            std::bind(&VisualOdometryNode::_newImageCallback, this, std::placeholders::_1));

    _pub_feature_image = this->create_publisher<sensor_msgs::msg::Image>(
            "/crazyflie/camera_features",
            10);
    
}

void VisualOdometryNode::_newImageCallback(const sensor_msgs::msg::Image::SharedPtr image)
{
    RCLCPP_DEBUG(this->get_logger(), "image received");
    // Convert the sensor_msgs::msg::Image to an OpenCV image
    cv::Mat cv_image = cv_bridge::toCvCopy(image)->image;

    // Create a black circle on the image
    int radius = 20;  // Adjust the radius as needed
    int x = 100;      // Adjust the circle's center coordinates as needed
    int y = 100;

    cv::circle(cv_image, cv::Point(x, y), radius, cv::Scalar(0, 0, 0), -1);

    // Convert the OpenCV image back to a sensor_msgs::msg::Image
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->image = cv_image;
    sensor_msgs::msg::Image image_with_features = *cv_ptr->toImageMsg();

    image_with_features.header.stamp = get_clock()->now();
    image_with_features.header.frame_id = "camera";
    image_with_features.encoding = "bgra8";
    _pub_feature_image->publish(image_with_features);

    _old_image = image;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
