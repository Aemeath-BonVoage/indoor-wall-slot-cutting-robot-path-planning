cmake_minimum_required(VERSION 3.8)
project(scr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/mecanum_controller.py
    scripts/arm_demo.py
    scripts/test_mecanum.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
