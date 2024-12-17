#===============================================================================
# Project Configuration
#===============================================================================

cmake_minimum_required(VERSION 3.8)
project(gd_px4_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_shared REQUIRED)

# Include Cpp "include" directory
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#===============================================================================
# C++ Stuff
#===============================================================================

#=============
# OpenCV camera node
#=============
add_library(
  video_capture_node SHARED
  src/video_capture.cpp
)
target_compile_definitions(
  video_capture_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  video_capture_node
  camera_calibration_parsers
  class_loader
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
  ros2_shared
)
rclcpp_components_register_nodes(video_capture_node "gd_video_cam::GDCameraNode")
set(node_plugins "${node_plugins}gd_video_cam::GDCameraNode;$<TARGET_FILE:video_capture_node>\n")

#=============
# OpenCV camera main, IPC=true
#=============

add_executable(
  initialize_cam_pipe
  src/initialize_cam_pipe.cpp
)
target_link_libraries(
  initialize_cam_pipe
  video_capture_node
)
ament_target_dependencies(
  initialize_cam_pipe
  rclcpp
)

#===============================================================================
# Project Build Stuff
#===============================================================================

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_gd_camera_node)

#=============
# Install
#=============

# Install nodes
install(
  TARGETS video_capture_node
  EXPORT export_gd_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS initialize_cam_pipe
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#===============================================================================
# Python Stuff
#===============================================================================

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  # scripts/py_node.py
  gd_px4_control/gesture_process_node.py
  DESTINATION lib/${PROJECT_NAME}
)

#===============================================================================
# Project Test Stuff
#===============================================================================

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
