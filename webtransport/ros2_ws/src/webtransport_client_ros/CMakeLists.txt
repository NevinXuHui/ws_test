cmake_minimum_required(VERSION 3.8)
project(webtransport_client_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)

add_executable(webtransport_node src/webtransport_node.cpp)
target_include_directories(webtransport_node PRIVATE include)
target_link_directories(webtransport_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH})
target_link_libraries(webtransport_node webtransport_client)
ament_target_dependencies(webtransport_node rclcpp std_msgs std_srvs)

install(TARGETS webtransport_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY lib/${ARCH}/ DESTINATION lib)

ament_package()
