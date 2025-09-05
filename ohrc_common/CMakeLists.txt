cmake_minimum_required(VERSION 3.5)
project(ohrc_common)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# set(singles
#   ft_filter
# )

# foreach(single ${singles})
#   add_executable(${single} src/${single}.cpp)
#   ament_target_dependencies(${single} rclcpp tf2_ros geometry_msgs std_srvs ohrc_common ohrc_msgs )
#   target_link_libraries(${single} magic_enum::magic_enum)
#   install(TARGETS ${single}
#     DESTINATION lib/${PROJECT_NAME})
# endforeach(single)



ament_auto_add_library(ohrc_common
  src/geometry_msgs_utility/Wrench.cpp
  src/geometry_msgs_utility/WrenchStamped.cpp
  src/geometry_msgs_utility/Point.cpp
  src/geometry_msgs_utility/Vector3.cpp
  src/geometry_msgs_utility/Twist.cpp
  src/filter_utility/butterworth.cpp
  src/filter_utility/bayes_normal_gamma.cpp
  src/filter_utility/kalman.cpp
)
target_link_libraries(ohrc_common magic_enum::magic_enum)
ament_export_include_directories(include)
ament_auto_package()