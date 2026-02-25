cmake_minimum_required(VERSION 3.8)
project(ursula)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/serial_command_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files (we'll add these later)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()