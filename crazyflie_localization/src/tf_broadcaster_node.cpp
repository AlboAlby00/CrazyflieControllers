#include "crazyflie_localization/tf_broadcaster_node.h"

TfBroadcasterNode::TfBroadcasterNode(const std::string & name) : Node(name)
{

}


int main(int argc, char const *argv[]){
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tf_broadcaster is running!" );
  auto node = std::make_shared<TfBroadcasterNode>("tf_broadcaster");
  rclcpp::spin(node);
  return 0;
}