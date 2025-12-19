cmake_minimum_required(VERSION 3.8)
project(websocket_client_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(homi_speech_interface REQUIRED)

# 获取架构
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}
)

add_executable(websocket_node src/websocket_node.cpp)

ament_target_dependencies(websocket_node
  rclcpp
  std_msgs
  std_srvs
  homi_speech_interface
)

target_link_libraries(websocket_node websocket_client)

install(TARGETS websocket_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
