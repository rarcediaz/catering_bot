cmake_minimum_required(VERSION 3.5)
project(my_bot)

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
# The same package is built on both the minimal Pi hardware image and the
# central navigation computer. Build the MPPI extension only where Nav2 is
# installed; the Pi does not run or require a local controller server.
find_package(nav2_mppi_controller QUIET)
if(nav2_mppi_controller_FOUND)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(xtensor REQUIRED)

  add_library(my_bot_mppi_critics SHARED
    src/final_approach_direction_critic.cpp
  )
  target_compile_features(my_bot_mppi_critics PUBLIC cxx_std_17)
  target_compile_options(my_bot_mppi_critics PUBLIC -fconcepts)
  target_include_directories(my_bot_mppi_critics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(my_bot_mppi_critics xtensor)
  ament_target_dependencies(my_bot_mppi_critics
    nav2_mppi_controller
    pluginlib
    rclcpp
  )

  pluginlib_export_plugin_description_file(
    nav2_mppi_controller
    my_bot_mppi_critics.xml
  )

  install(
    TARGETS my_bot_mppi_critics
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )
else()
  message(STATUS "Nav2 MPPI is unavailable; skipping central navigation critic build.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    runtime_contract_test
    test/test_runtime_contract.py
  )
  ament_add_pytest_test(
    safety_logic_test
    test/test_safety_logic.py
  )
  ament_add_pytest_test(
    nav2_startup_gate_test
    test/test_nav2_startup_gate.py
  )
  ament_add_pytest_test(
    wifi_startup_test
    test/test_wifi_startup.py
  )
  ament_add_pytest_test(
    wifi_provisioning_test
    test/test_wifi_provisioning.py
  )
  if(TARGET my_bot_mppi_critics)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(
      final_approach_direction_latch_test
      test/test_final_approach_direction_latch.cpp
    )
    target_compile_features(final_approach_direction_latch_test PUBLIC cxx_std_17)
    target_include_directories(final_approach_direction_latch_test PRIVATE include)
    target_link_libraries(final_approach_direction_latch_test my_bot_mppi_critics)
  endif()
endif()

install(
  DIRECTORY behavior_trees config description docs launch maps systemd wifi_provisioning_ui worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/xbox_controller.py
    scripts/safety_node.py
    scripts/robot_health_node.py
    scripts/scan_canonicalizer.py
    scripts/heartbeat_node.py
    scripts/nav2_startup_gate.py
    scripts/generate_cyclonedds_config.py
    scripts/generate_wifi_ap_config.sh
    scripts/install_wifi_provisioning_service.sh
    scripts/preview_wifi_provisioning_ui.py
    scripts/start_robot_stack.sh
    scripts/wifi_startup.py
    scripts/wifi_provisioning_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
