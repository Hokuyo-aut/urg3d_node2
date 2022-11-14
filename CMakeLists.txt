cmake_minimum_required(VERSION 3.5)
project(urg3d_node2)

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
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(laser_proc REQUIRED)
find_package(std_msgs REQUIRED)

set(URG_LIBRARY_DIR urg3d_library)
set(URG_LIBRARY_C_INCLUDE_DIR ${URG_LIBRARY_DIR}/include)
set(URG_LIBRARY_SRC_DIR ${URG_LIBRARY_DIR}/src)

include_directories(include ${URG_LIBRARY_C_INCLUDE_DIR})

add_library(urg3d_c SHARED
  ${URG_LIBRARY_SRC_DIR}/urg3d_connection.c
  ${URG_LIBRARY_SRC_DIR}/urg3d_ring_buffer.c
  ${URG_LIBRARY_SRC_DIR}/urg3d_sensor.c
  ${URG_LIBRARY_SRC_DIR}/urg3d_tcpclient.c
  ${URG_LIBRARY_SRC_DIR}/urg3d_ticks.c
)

add_library(urg3d_node2 SHARED src/urg3d_node2.cpp)
ament_target_dependencies(urg3d_node2 rclcpp rclcpp_components rclcpp_lifecycle lifecycle_msgs sensor_msgs diagnostic_updater laser_proc)
rclcpp_components_register_node(urg3d_node2
  PLUGIN "urg3d_node2::Urg3dNode2"
  EXECUTABLE urg3d_node2_node)
target_link_libraries(urg3d_node2 urg3d_c)

install(TARGETS
  urg3d_node2 urg3d_c
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  ament_add_gtest(urg3d_node2_test src/urg3d_node2.cpp test/urg3d_node2_test.cpp TIMEOUT 200)
  ament_target_dependencies(urg3d_node2_test rclcpp rclcpp_components rclcpp_lifecycle lifecycle_msgs sensor_msgs diagnostic_updater laser_proc)
  target_link_libraries(urg3d_node2_test urg3d_c)
endif()

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
