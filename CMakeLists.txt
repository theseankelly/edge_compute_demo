cmake_minimum_required(VERSION 3.5)
project(edge_compute_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ifm3d_ros2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(ifm3d 0.17.0 CONFIG REQUIRED COMPONENTS
  camera
  framegrabber
  image
  )
include_directories(${PCL_INCLUDE_DIRS})

add_executable(compute_mean_depth src/compute_mean_depth.cpp)
ament_target_dependencies(compute_mean_depth
  rclcpp
  std_msgs
  sensor_msgs
  lifecycle_msgs
  cv_bridge
  ifm3d_ros2
)
target_link_libraries(compute_mean_depth
  ifm3d::camera
  ifm3d::framegrabber
  ifm3d::image
)

add_executable(preprocess_data src/preprocess_data.cpp)
ament_target_dependencies(preprocess_data
  rclcpp
  std_msgs
  sensor_msgs
  lifecycle_msgs
  pcl_conversions
  ifm3d_ros2
)
target_link_libraries(preprocess_data
  ifm3d::camera
  ifm3d::framegrabber
  ifm3d::image
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

install(TARGETS
  compute_mean_depth
  preprocess_data
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
