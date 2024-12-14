# PointCloud2解析
PointCloud2是ros中传递点云数据的标准消息。本文通过解析PointCloud2数据，了解ros2中消息体和c++数据格式的转换。
## msg的定义
```msg
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.
#
# The point cloud data may be organized 2d (image-like) or 1d (unordered).
# Point clouds organized as 2d images may be produced by camera depth sensors
# such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields
	uint8 INT8    = 1
	uint8 UINT8   = 2
	uint8 INT16   = 3
	uint8 UINT16  = 4
	uint8 INT32   = 5
	uint8 UINT32  = 6
	uint8 FLOAT32 = 7
	uint8 FLOAT64 = 8
	string name      #
	uint32 offset    #
	uint8  datatype  #
	uint32 count     #

bool    is_bigendian # Is this data bigendian? 大端序
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```
1. 上诉消息体中，PointField是一个单独的类型，fields数组中，offset是数据在data中的偏移量，count是数据个数，datatype是数据类型。大写表示常量，在实例化c++代码中，实例化为enum类型或常量。
2. header是消息头，包含时间戳和坐标系ID。
3. height和width 规定了数据的行列，height=1,表示数据无序。
4. 根据PointField数据，确定每个数据的构成，真实数据存储在data中。is_bigendian表示数据是否是大端序，point_step表示一个点的大小，row_step表示一行的大小。
5. is_dense表示数据是否包含无效数据。

## c++数据格式
上述msg对应的实现，header对应不上。
```c++
  struct PCLPointField
  {
    std::string name;

    uindex_t offset = 0;
    std::uint8_t datatype = 0;
    uindex_t count = 0;

    enum PointFieldTypes { INT8 = traits::asEnum_v<std::int8_t>,
                           UINT8 = traits::asEnum_v<std::uint8_t>,
                           INT16 = traits::asEnum_v<std::int16_t>,
                           UINT16 = traits::asEnum_v<std::uint16_t>,
                           INT32 = traits::asEnum_v<std::int32_t>,
                           UINT32 = traits::asEnum_v<std::uint32_t>,
                           FLOAT32 = traits::asEnum_v<float>,
                           FLOAT64 = traits::asEnum_v<double>};

  public:
    using Ptr = shared_ptr< ::pcl::PCLPointField>;
    using ConstPtr = shared_ptr<const ::pcl::PCLPointField>;
  }; // struct PCLPointField

  struct PCLHeader
  {
    /** \brief Sequence number */
    std::uint32_t seq = 0;
    /** \brief A timestamp associated with the time when the data was acquired
      *
      * The value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
      */
    std::uint64_t stamp = 0;
    /** \brief Coordinate frame ID */
    std::string frame_id;

    using Ptr = shared_ptr<PCLHeader>;
    using ConstPtr = shared_ptr<const PCLHeader>;
  }; // struct PCLHeader

  struct PCL_EXPORTS PCLPointCloud2
  {
    ::pcl::PCLHeader header;

    uindex_t height = 0;
    uindex_t width = 0;

    std::vector<::pcl::PCLPointField>  fields;

    static_assert(BOOST_ENDIAN_BIG_BYTE || BOOST_ENDIAN_LITTLE_BYTE, "unable to determine system endianness");
    std::uint8_t is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    uindex_t point_step = 0;
    uindex_t row_step = 0;

    std::vector<std::uint8_t> data;

    std::uint8_t is_dense = 0;
    }
```

## 使用
```cmake
# 点云相关
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# 包含PCL头文件路径
include_directories(${PCL_INCLUDE_DIRS})
add_executable(pointCloudTrans src/pointCloud.cpp)
ament_target_dependencies(pointCloudTrans rclcpp pcl_conversions PCL)
# 安装到工作空间的目录下
install(TARGETS
  pointCloudTrans
  DESTINATION lib/${PROJECT_NAME}
)
```
```xml
  <depend>pcl_conversions</depend>
  <depend>pcl_ros</depend> <!-- 或者是 pcl_msgs -->
```
```c++
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
```