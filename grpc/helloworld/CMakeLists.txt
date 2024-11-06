cmake_minimum_required(VERSION 3.15)

project(helloWorld CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# 设置 vcpkg 作为包管理器（如果使用 vcpkg）
#if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed")
#   set(CMAKE_TOOLCHAIN_FILE "D:/user/zxm/vcpkg-master/scripts/buildsystems/vcpkg.cmake")
#endif()

set(CMAKE_PREFIX_PATH "D:/user/zxm/vcpkg-master/helloworld/vcpkg_installed/x64-windows/share" "${CMAKE_PREFIX_PATH}")
message(STATUS "${CMAKE_PREFIX_PATH}")
find_package(fmt CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(OpenSSL CONFIG REQUIRED)
find_package(ZLIB REQUIRED)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


# 包含 gRPC 和 Protobuf 的头文件
include_directories(${gRPC_INCLUDE_DIRS})
include_directories(${Protobuf_INCLUDE_DIRS})

add_executable(helloWorld main.cxx)

target_link_libraries(helloWorld
  PRIVATE
    fmt::fmt ZLIB::ZLIB
)

add_subdirectory(src)