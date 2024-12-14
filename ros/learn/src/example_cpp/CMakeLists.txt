cmake_minimum_required(VERSION 3.8)
project(example_cpp)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# 添加编译选项，仅显示错误信息
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-w)  # 关闭所有警告
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# 标准消息
find_package(std_msgs REQUIRED)
# 自定义消息
find_package(example_ros2_interfaces REQUIRED)

# 点云相关
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# 包含PCL头文件路径
include_directories(${PCL_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(node_01 src/node1.cpp)
ament_target_dependencies(node_01 rclcpp std_msgs example_ros2_interfaces)
add_executable(node_02 src/node2.cpp)
ament_target_dependencies(node_02 rclcpp std_msgs example_ros2_interfaces)

add_executable(pointCloudTrans src/pointCloud.cpp)
ament_target_dependencies(pointCloudTrans rclcpp pcl_conversions PCL)
# 安装到工作空间的目录下
install(TARGETS
  node_01
  node_02
  pointCloudTrans
  DESTINATION lib/${PROJECT_NAME}
)

