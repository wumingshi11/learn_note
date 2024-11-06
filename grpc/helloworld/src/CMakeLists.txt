

# 定义 proto 文件
set(PROTO_FILES service.proto file_transfer.proto)
#set(CPP_SOURCES greeter_server.cc greeter_client.cc)
# 生成 gRPC 和 Protobuf 文件
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
#grpc_generate_cpp(GRPC_SRCS GRPC_HDRS ${PROTO_FILES})
# 生成 gRPC 文件
#message(STATUS  "grpc: ${gRPC_CPP_PLUGIN_EXECUTABLE}")
#add_custom_command(
#    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/service_grpc.pb.cc
#           ${CMAKE_CURRENT_BINARY_DIR}/service_grpc.pb.h
 #   COMMAND ${gRPC_CPP_PLUGIN_EXECUTABLE}
#    ARGS --grpc_out=${CMAKE_CURRENT_BINARY_DIR} --plugin=protoc-gen-grpc=${gRPC_CPP_PLUGIN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/service.proto
#    DEPENDS ${PROTO_FILES}
#)

# 添加生成的源文件
#add_library(proto_files ${PROTO_SRCS} ${PROTO_HDRS} ${CMAKE_CURRENT_BINARY_DIR}/service_grpc.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/service_grpc.pb.h)


# 手动生成 gRPC 代码
set(TOOL_PATH ${Protobuf_DIR}/../../tools)
set(gRPC_PROTOC_EXECUTABLE "${TOOL_PATH}/protobuf/protoc")
set(gRPC_PLUGIN_EXECUTABLE "${TOOL_PATH}/grpc/grpc_cpp_plugin.exe")
message(STATUS  "grpc: ${gRPC_PROTOC_EXECUTABLE}")
message(STATUS  "grpc_exec: ${gRPC_PLUGIN_EXECUTABLE}")
foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    set(GRPC_CPP_OUT "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.cc")
    set(GRPC_H_OUT "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.h")
    add_custom_command(
        OUTPUT ${GRPC_CPP_OUT} ${GRPC_H_OUT}
        COMMAND ${gRPC_PROTOC_EXECUTABLE}
        ARGS --grpc_out=${CMAKE_CURRENT_BINARY_DIR}
             --plugin=protoc-gen-grpc=${gRPC_PLUGIN_EXECUTABLE}
             -I${CMAKE_CURRENT_SOURCE_DIR}
             ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Running gRPC protocol buffer compiler on ${PROTO_FILE}"
        VERBATIM
    )
    list(APPEND GRPC_SRCS ${GRPC_CPP_OUT})
    list(APPEND GRPC_HDRS ${GRPC_H_OUT})
endforeach()

# 添加源文件列表
set(SOURCES ${PROTO_SRCS} ${GRPC_SRCS} ${PROTO_HDRS} ${GRPC_HDRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_library(librpc ${SOURCES})
target_link_libraries(librpc gRPC::grpc++ gRPC::grpc++_reflection gRPC::gpr gRPC::grpc gRPC::grpc++ protobuf::libprotoc protobuf::libprotobuf protobuf::libprotobuf-lite)

# 添加服务器可执行文件
add_executable(grpc_server server.cxx)
target_link_libraries(grpc_server  librpc)

# 添加客户端可执行文件
add_executable(grpc_client client.cxx)
target_link_libraries(grpc_client  librpc)