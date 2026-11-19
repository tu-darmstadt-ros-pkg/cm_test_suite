cmake_minimum_required(VERSION 3.16)
project(cm_test_suite_legacy LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(cm_test_suite REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(rclcpp REQUIRED)

  # The suite registers its tests at runtime, so the executable has its own main
  ament_add_gtest(test_legacy_cm test/test_legacy_cm.cpp SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 1800)
  target_link_libraries(test_legacy_cm
    cm_test_suite::cm_test_suite
    controller_manager::controller_manager
    rclcpp::rclcpp
    ${controller_manager_msgs_TARGETS}
  )
endif()

ament_package()
