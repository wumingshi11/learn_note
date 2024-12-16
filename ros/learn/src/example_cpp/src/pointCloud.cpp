#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> // 用于ROS与PCL之间的转换
#include <sensor_msgs/msg/point_cloud2.hpp>
int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::msg::PointCloud2 ros_point_cloud2_msg;
  // ... (这里填充ros_point_cloud2_msg)
  ros_point_cloud2_msg.header.frame_id = "map";

  // 将ROS PointCloud2消息转换为PCL点云
  pcl::fromROSMsg(ros_point_cloud2_msg, *cloud);
  // 将PCL点云转换为ROS PointCloud2消息
  pcl::toROSMsg(*cloud, ros_point_cloud2_msg);
  return 0;
}
