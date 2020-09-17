cmake_minimum_required(VERSION 3.5)

project(drone)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)


add_executable(${PROJECT_NAME}
  src/drone_main.cpp
  src/drone.cpp
  src/auto_pilot_px4.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "sensor_msgs"
  "nav_msgs"
  "px4_msgs"
)

install(  TARGETS
            ${PROJECT_NAME}
          DESTINATION
            lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Auto pilot tests
  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_AUTO_PILOT test_auto_pilot)
  ament_add_gtest_executable(${TEST_AUTO_PILOT}
    src/auto_pilot_px4.cpp
    src/drone.cpp
    test/test_auto_pilot_px4.cpp
  )
  target_include_directories(${TEST_AUTO_PILOT} PRIVATE
    ${CMAKE_SOURCE_DIR}
  )
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Cannot use -Wpedantic when using GoogleMock (spews many warnings about the macros used).
    target_compile_options(${TEST_AUTO_PILOT} PRIVATE -Wall -Wextra)
  endif()
  ament_target_dependencies(${TEST_AUTO_PILOT}
    "rclcpp"
    "rclcpp_action"
    "std_msgs"
    "sensor_msgs"
    "nav_msgs"
    "px4_msgs"
  )
endif()  # BUILD_TESTING

ament_package()
