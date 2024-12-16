#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_ros2_interfaces/msg/self_test_msg.hpp"

int main(int argc, char **argv) {
  /* 初始化rclcpp  */
  rclcpp::init(argc, argv);
  /*产生一个node_01的节点*/
  auto node = std::make_shared<rclcpp::Node>("node_01");
  // 打印一句自我介绍
  RCLCPP_INFO(node->get_logger(), "node_01节点已经启动.");
  /* 运行节点，并检测退出信号 Ctrl+C*/

  // 给节点增加一个send topic功能, 10为Qos的默认构造函数，保存10最后消息
  auto publish = node->create_publisher<std_msgs::msg::String>("topic_01", 10);
  size_t count = 0;
  // 动作是异步执行的
  auto timer = node->create_wall_timer(std::chrono::milliseconds(500),
                                       [&publish, &count,&node]() {
                                         std_msgs::msg::String m;
                                         m.data = std::string("send : ") + std::to_string(count++);
                                         publish->publish(m);
                                         RCLCPP_INFO(node->get_logger(), "send: '%s'", m.data.c_str());
                                       });
   size_t count2 = 0;
   auto publish2 = node->create_publisher<example_ros2_interfaces::msg::SelfTestMsg>("topic_02", 10);
   auto timer2 = node->create_wall_timer(std::chrono::milliseconds(1000),
                                       [&publish2, &count2,&node]() {
                                         example_ros2_interfaces::msg::SelfTestMsg m;
                                         m.status = count2++;
                                         m.status_stop = 0;
                                         publish2->publish(m);
                                       }
   );

  // topic2 自定义消息
  // auto publish2 = node->create_publisher<msg_demo::msg::CustomMsg>("topic_02", 10);

  rclcpp::spin(node);
  /* 停止运行 */
  rclcpp::shutdown();
  return 0;
}
