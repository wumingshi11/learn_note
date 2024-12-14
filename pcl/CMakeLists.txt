cmake_minimum_required(VERSION 3.8)
project(example_cpl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(PCL REQUIRED COMPONENTS common io kdtree filters keypoints registration visualization sample_consensus)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
find_package(GLog REQUIRED)
# 包含PCL头文件路径
include_directories(${PCL_INCLUDE_DIRS})
# 

add_subdirectory(src)