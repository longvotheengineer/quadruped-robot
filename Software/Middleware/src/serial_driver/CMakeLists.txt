cmake_minimum_required(VERSION 3.8)
project(serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 1. Find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Define the executable
add_executable(serial_driver_node src/serial_driver.cpp)

# 3. Link libraries (Wire the headers)
ament_target_dependencies(serial_driver_node rclcpp std_msgs)

# 4. Install (Copy the executable to 'lib/leg_controller' so ROS can find it)
install(TARGETS
  serial_driver_node
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