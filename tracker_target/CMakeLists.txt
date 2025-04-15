cmake_minimum_required(VERSION 3.8)
project(tracker_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(spin_flag_interfaces REQUIRED)
find_package(pb_rm_interfaces REQUIRED)


add_executable(tracker_target src/tracker_target.cpp)
ament_target_dependencies(tracker_target
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
)
add_executable(pub_tracker_target src/pub_tracker_target.cpp)
ament_target_dependencies(pub_tracker_target
  rclcpp
  geometry_msgs
)
add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)
add_executable(tf_republisher src/tf_republisher.cpp)
ament_target_dependencies(tf_republisher
  rclcpp
  tf2_msgs
  nav2_msgs
  geometry_msgs
)
add_executable(sub_msg src/sub_msg.cpp)
ament_target_dependencies(sub_msg
  rclcpp
  spin_flag_interfaces
  pb_rm_interfaces
  geometry_msgs
  auto_aim_interfaces
)


install(TARGETS
  tracker_target
  pub_tracker_target
  waypoint_follower
  tf_republisher
  sub_msg
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
