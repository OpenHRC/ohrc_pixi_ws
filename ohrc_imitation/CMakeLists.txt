cmake_minimum_required(VERSION 3.5)
project(ohrc_imitation)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ohrc_control REQUIRED)
find_package(ohrc_msgs REQUIRED)
find_package(ohrc_common REQUIRED)
find_package(ohrc_teleoperation REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(geometry_msgs REQUIRED)


find_package(trajectory_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(urdf REQUIRED)

find_package(magic_enum CONFIG REQUIRED)

find_package(proxsuite REQUIRED)

set(interfaces
  marker
  state_topic
  twist_topic
  pose_topic
)


foreach(interface ${interfaces})
    add_executable(${interface}_imitation src/${interface}_imitation.cpp)

    ament_target_dependencies(${interface}_imitation rclcpp ohrc_control ohrc_msgs ohrc_common ohrc_teleoperation interactive_markers visualization_msgs control_msgs geometry_msgs std_srvs tf2_kdl urdf tf2_eigen_kdl urdf)

    target_include_directories(${interface}_imitation
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${interface}_imitation magic_enum::magic_enum proxsuite::proxsuite)

    install(TARGETS ${interface}_imitation
    DESTINATION lib/${PROJECT_NAME}
    )
endforeach(interface)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY
#  config
#  DESTINATION share/${PROJECT_NAME}
# )
ament_package()