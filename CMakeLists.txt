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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
    wifi_startup_test
    test/test_wifi_startup.py
  )
  ament_add_pytest_test(
    wifi_provisioning_test
    test/test_wifi_provisioning.py
  )
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
