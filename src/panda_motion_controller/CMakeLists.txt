cmake_minimum_required(VERSION 3.8)
project(panda_motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(control_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

add_library(action_client SHARED
  src/trajectory_selector.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
# add_executable(trajectory_selector src/trajectory_selector.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action rclcpp_components
std_msgs moveit_core moveit_ros_planning_interface rosidl_default_generators franka_msgs 
sensor_msgs control_msgs rclcpp_components action_tutorials_interfaces)
rclcpp_components_register_node(action_client PLUGIN "action_trajectory_selector::PandaMotionController" EXECUTABLE trajectory_selector)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(spacebar_publisher src/spacebar_publisher.cpp)
ament_target_dependencies(spacebar_publisher rclcpp std_msgs)
install(TARGETS spacebar_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(set_workspace src/set_workspace.cpp)
ament_target_dependencies(
  set_workspace
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  std_msgs
)
install(TARGETS set_workspace DESTINATION lib/${PROJECT_NAME})

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

#install(TARGETS
#  trajectory_selector
#  DESTINATION lib/${PROJECT_NAME})

ament_package()
