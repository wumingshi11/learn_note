#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_ros2_interfaces/msg/self_test_msg.hpp"
int main(int argc, char const *argv[]) {
  //创建节点
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("node2");
  RCLCPP_INFO(node->get_logger(), "node2 start");

  // 创建话题接收者
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "topic_01", 10, [](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%s'",
                    msg->data.c_str());
      });

   
   auto sub2 = node->create_subscription<example_ros2_interfaces::msg::SelfTestMsg>(
      "topic_02", 10, [&node](example_ros2_interfaces::msg::SelfTestMsg::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%u'",
                    msg->status);
        RCLCPP_INFO(node->get_logger(), "statu: '%u'", msg->status_stop);
      });

  // 执行节点，并监听退出消息
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
