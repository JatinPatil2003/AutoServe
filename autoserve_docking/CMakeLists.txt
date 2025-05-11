cmake_minimum_required(VERSION 3.8)
project(autoserve_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters io common)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create executable
add_executable(docking_server src/main.cpp src/docking_server.cpp src/controller.cpp src/smooth_control_law.cpp src/dock_detector.cpp)

# Declare dependencies
ament_target_dependencies(docking_server
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  angles
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(docking_server
  ${PCL_LIBRARIES}
  tf2::tf2
  tf2_eigen::tf2_eigen
)

# Install target and include directories
install(TARGETS
  docking_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
