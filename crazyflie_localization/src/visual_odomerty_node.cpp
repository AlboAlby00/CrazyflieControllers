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

void VisualOdometryNode::_newImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "image received");
    // Convert the sensor_msgs::msg::Image to an OpenCV image
    cv::Mat new_image = cv_bridge::toCvCopy(msg)->image;
    assert(new_image.data != nullptr);

    if(_old_image.data == nullptr ) 
    {
        _old_image = new_image;
        return;
    }


    // select features

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    //−− detect Oriented FAST
    detector->detectAndCompute(_old_image, cv::noArray(), keypoints_1, descriptors_1);
    detector->detectAndCompute(new_image, cv::noArray(), keypoints_2, descriptors_2);

    cv::Mat new_image_with_keypoints;
    cv::drawKeypoints(new_image, keypoints_1, new_image_with_keypoints, cv::Scalar(0, 0, 255), 
            cv::DrawMatchesFlags::DEFAULT);


    // Convert the OpenCV image back to a sensor_msgs::msg::Image
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->image = new_image_with_keypoints;
    sensor_msgs::msg::Image image_with_features = *cv_ptr->toImageMsg();


    image_with_features.header.stamp = get_clock()->now();
    image_with_features.header.frame_id = "camera";
    image_with_features.encoding = "bgr8";
    _pub_feature_image->publish(image_with_features);

    _old_image = new_image;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
