#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class TfBroadcasterNode : public rclcpp::Node{

    public:
        TfBroadcasterNode(const std::string & name);

    private:
        rclcpp::TimerBase::SharedPtr _transform_timer;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _pose_subscriber;
    

};
