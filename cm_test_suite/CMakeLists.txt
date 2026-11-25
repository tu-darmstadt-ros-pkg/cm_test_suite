cmake_minimum_required(VERSION 3.16)
project(cm_test_suite LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

set(THIS_PACKAGE_DEPENDS
    ament_index_cpp
    controller_interface
    GTest
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle)
find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# Shared, so the plugins loaded by pluginlib and the harness see the same trace
# and fault board
add_library(cm_test_suite_core SHARED src/faults.cpp src/rig.cpp src/trace.cpp)
target_include_directories(
  cm_test_suite_core PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include/cm_test_suite>)

add_library(
  cm_test_suite_plugins SHARED src/controller_tracer.cpp
                               src/trace_controllers.cpp src/trace_system.cpp)
target_link_libraries(
  cm_test_suite_plugins
  PUBLIC cm_test_suite_core controller_interface::controller_interface
         hardware_interface::hardware_interface pluginlib::pluginlib
         rclcpp_lifecycle::rclcpp_lifecycle)
# The hook is a library of its own that only the plugin links, so it unloads
# with the plugin
add_library(cm_test_suite_pre_shutdown_hook SHARED src/pre_shutdown_hook.cpp)
target_link_libraries(cm_test_suite_pre_shutdown_hook PUBLIC rclcpp::rclcpp)
add_library(cm_test_suite_pre_shutdown_plugin SHARED
            src/pre_shutdown_controller.cpp)
target_link_libraries(
  cm_test_suite_pre_shutdown_plugin
  PUBLIC cm_test_suite_pre_shutdown_hook
         controller_interface::controller_interface pluginlib::pluginlib)
add_library(cm_test_suite_pre_shutdown_hardware SHARED
            src/pre_shutdown_sensor.cpp)
target_link_libraries(
  cm_test_suite_pre_shutdown_hardware
  PUBLIC cm_test_suite_pre_shutdown_hook hardware_interface::hardware_interface
         pluginlib::pluginlib)
pluginlib_export_plugin_description_file(controller_interface
                                         controller_plugins.xml)
pluginlib_export_plugin_description_file(hardware_interface
                                         hardware_plugins.xml)

# Static, so that linking it replaces operator new in the test executable for
# the allocation count
add_library(
  cm_test_suite STATIC
  src/control_loop.cpp
  src/event_summary.cpp
  src/generator.cpp
  src/operation.cpp
  src/oracle_state.cpp
  src/oracle_steps.cpp
  src/rt_allocations.cpp
  src/runner.cpp
  src/script.cpp
  src/shrink.cpp
  src/status_sampler.cpp
  src/suite.cpp)
set_target_properties(cm_test_suite PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(
  cm_test_suite PUBLIC cm_test_suite_core ament_index_cpp::ament_index_cpp
                       GTest::gtest)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_cm_test_suite
    test/test_rig.cpp
    test/test_trace.cpp
    test/test_script.cpp
    test/test_oracle.cpp
    test/test_shrink.cpp
    test/test_tiers.cpp)
  target_link_libraries(test_cm_test_suite cm_test_suite)
  target_compile_definitions(
    test_cm_test_suite PRIVATE SCRIPTS_DIR="${PROJECT_SOURCE_DIR}/scripts")

  # Exercises internal harness parts directly, so it sees the private headers
  ament_add_gtest(test_cm_test_suite_harness test/test_harness.cpp TIMEOUT 300)
  target_link_libraries(
    test_cm_test_suite_harness cm_test_suite cm_test_suite_plugins
    rclcpp_lifecycle::rclcpp_lifecycle)
  target_include_directories(test_cm_test_suite_harness PRIVATE src)
  target_compile_definitions(
    test_cm_test_suite_harness
    PRIVATE SCRIPTS_DIR="${PROJECT_SOURCE_DIR}/scripts")

  ament_add_gtest(test_cm_test_suite_all_async test/test_all_async.cpp ENV
                  CM_FUZZ_ALL_ASYNC=1)
  target_link_libraries(test_cm_test_suite_all_async cm_test_suite
                        cm_test_suite_plugins)
  target_include_directories(test_cm_test_suite_all_async PRIVATE src)
endif()

install(DIRECTORY include/ DESTINATION include/cm_test_suite)
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(
  TARGETS cm_test_suite_core cm_test_suite_plugins
          cm_test_suite_pre_shutdown_hook cm_test_suite_pre_shutdown_plugin
          cm_test_suite_pre_shutdown_hardware cm_test_suite
  EXPORT export_cm_test_suite
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
ament_export_targets(export_cm_test_suite HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
