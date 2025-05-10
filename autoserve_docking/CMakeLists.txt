cmake_minimum_required(VERSION 3.8)
project(autoserve_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(dock_detector_node src/dock_detector.cpp)
ament_target_dependencies(dock_detector_node rclcpp sensor_msgs geometry_msgs pcl_conversions)
target_link_libraries(dock_detector_node ${PCL_LIBRARIES})

# Build the main executable
add_executable(docking_server src/main.cpp src/docking_server.cpp src/controller.cpp src/smooth_control_law.cpp)
ament_target_dependencies(docking_server rclcpp geometry_msgs angles tf2 tf2_ros tf2_geometry_msgs std_srvs)

install(TARGETS
  dock_detector_node
  docking_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
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
