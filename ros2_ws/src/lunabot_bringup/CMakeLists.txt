cmake_minimum_required(VERSION 3.8)
project(lunabot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install directories
install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS scripts/zone_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zone_manager_node
)

install(
  PROGRAMS scripts/environmental_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME environmental_sensor_node
)

install(
  PROGRAMS scripts/hazard_safety_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hazard_safety_node
)

install(
  PROGRAMS scripts/stereo_depth_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stereo_depth_node
)

install(
  PROGRAMS scripts/terramechanics_ml_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME terramechanics_ml_node
)

ament_package()
