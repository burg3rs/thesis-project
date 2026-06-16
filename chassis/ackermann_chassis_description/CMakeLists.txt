cmake_minimum_required(VERSION 3.8)
project(ackermann_chassis_description)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ── Hardware plugin library (POSIX serial, no external dep) ───
add_library(ackermann_arduino_hw SHARED
  src/ArduinoSerialHardware.cpp
)

target_include_directories(ackermann_arduino_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ackermann_arduino_hw
  rclcpp
  hardware_interface
  pluginlib
)

# Export the plugin description to pluginlib
pluginlib_export_plugin_description_file(hardware_interface ackermann_chassis_arduino.xml)

# ── Install ────────────────────────────────────────────────────
install(TARGETS ackermann_arduino_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ackermann_chassis_arduino.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests (optional) ───────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
