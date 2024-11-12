# ros2
## 安装
https://fishros.com/d2lros2foxy/#/chapt2/2.3ROS2%E7%9A%84%E5%AE%89%E8%A3%85
## 概念
1. 工作空间 ： 一个含有src文件夹的目录
2. 功能包 ： 存放节点的地方，分为3中类型
- ament_python
- cmake
- ament_cmake  cmake的增强版
3. 节点
## 常用命令行
```sh
# 运行功能包下的节点
ros2 run <package_name> <executable_name>
# 查看功能包
ros2 pkg list
# 查看节点 （运行中的）
ros2 node list
# 查看节点详情
ros2 node info <node_name>
# 包相关命令
ros pkg create  <pkg name>      Create a new ROS2 package
ros pkg executables <pkg name>   Output a list of package specific executables
ros pkg list <pkg name>         Output a list of available packages
ros pkg prefix <pkg name>       Output the prefix path of a package
ros pkg xml <pkg name>          Output the XML of the package manifest or a specific tag
```