cmake_minimum_required(VERSION 3.8)
project(nav2_map_io_lab)

find_package(ament_cmake REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(map_io_demo src/map_io_demo.cpp)
target_compile_features(map_io_demo PUBLIC cxx_std_17)
ament_target_dependencies(map_io_demo nav2_map_server nav_msgs rclcpp)

install(TARGETS map_io_demo DESTINATION lib/${PROJECT_NAME})

ament_package()
