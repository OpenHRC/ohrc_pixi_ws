cmake_minimum_required(VERSION 3.5)
project(ohrc_tools)

## Compile as C++17
add_compile_options(-std=c++17)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ohrc_common REQUIRED)
find_package(ohrc_msgs REQUIRED)

find_package(magic_enum CONFIG REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${ohrc_common_INCLUDE_DIRS}
  ${ohrc_msgs_INCLUDE_DIRS}
)

set(singles
  ft_filter
  # body_state_visualizer
  # body_state_prediction
  # k4abt_body_state_publisher
)

foreach(single ${singles})
  add_executable(${single} src/${single}.cpp)
  ament_target_dependencies(${single} rclcpp tf2_ros geometry_msgs std_srvs ohrc_common ohrc_msgs )
  target_link_libraries(${single} magic_enum::magic_enum)
  install(TARGETS ${single}
    DESTINATION lib/${PROJECT_NAME})
endforeach(single)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()