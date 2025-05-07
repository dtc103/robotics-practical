cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(obstacle_avoidance_lib
  src/obstacle_avoidance/obstacle_avoidance.cpp
)

ament_target_dependencies(obstacle_avoidance_lib
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
)

add_library(potential_field_lib
  src/potential_field/potential_field.cpp
)
ament_target_dependencies(potential_field_lib
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
)

add_executable(obstacle_avoidance src/main.cpp)
target_link_libraries(obstacle_avoidance
  obstacle_avoidance_lib
  potential_field_lib
)

ament_target_dependencies(obstacle_avoidance
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
)

install(TARGETS
  obstacle_avoidance_lib
  potential_field_lib
  obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
