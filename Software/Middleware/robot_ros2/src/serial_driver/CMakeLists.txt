cmake_minimum_required(VERSION 3.8)
project(serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ───────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── SCServo Library (official Feetech driver) ─────────────────────────────

set(SCSERVO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SCServo_Linux/SCServo_Linux_220329/SCServo_Linux)

# Collect all SCServo source files
file(GLOB SCSERVO_SOURCES ${SCSERVO_DIR}/*.cpp)
file(GLOB SCSERVO_HEADERS ${SCSERVO_DIR}/*.h)

# Build SCServo as a static library within this package
add_library(scservo_lib STATIC ${SCSERVO_SOURCES})
target_include_directories(scservo_lib PUBLIC ${SCSERVO_DIR})

# Suppress warnings from the third-party library
target_compile_options(scservo_lib PRIVATE -w)

# ── serial_driver_node executable ─────────────────────────────────────────

add_executable(serial_driver_node src/serial_driver.cpp)

target_include_directories(serial_driver_node PRIVATE ${SCSERVO_DIR})
target_link_libraries(serial_driver_node scservo_lib)

ament_target_dependencies(serial_driver_node
  rclcpp
  std_msgs
  sensor_msgs
)

# ── calibrate_limits tool ─────────────────────────────────────────────────

add_executable(calibrate_limits src/calibrate_limits.cpp)
target_include_directories(calibrate_limits PRIVATE ${SCSERVO_DIR})
target_link_libraries(calibrate_limits scservo_lib)

# ── servo_calibrator tool ─────────────────────────────────────────────────

add_executable(servo_calibrator src/servo_calibrator.cpp)
target_include_directories(servo_calibrator PRIVATE ${SCSERVO_DIR})
target_link_libraries(servo_calibrator scservo_lib)

# ── Install ───────────────────────────────────────────────────────────────

install(TARGETS
  serial_driver_node
  calibrate_limits
  servo_calibrator
  DESTINATION lib/${PROJECT_NAME}
)

# ── Testing ───────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()