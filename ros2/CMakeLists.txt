cmake_minimum_required(VERSION 3.8)
project(websocket_client_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../cpp
)

# 库文件路径
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../cpp
)

# 创建可执行文件
add_executable(websocket_node src/websocket_node.cpp)

ament_target_dependencies(websocket_node
  rclcpp
  std_msgs
)

target_link_libraries(websocket_node
  websocket_client
)

install(TARGETS
  websocket_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
