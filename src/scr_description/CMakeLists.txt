cmake_minimum_required(VERSION 3.8)
project(scr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 基于力的平面运动插件（替代 gazebo_ros_planar_move，见 src/scr_planar_move.cpp）
add_library(scr_planar_move SHARED src/scr_planar_move.cpp)
ament_target_dependencies(scr_planar_move
  gazebo_dev gazebo_ros rclcpp geometry_msgs nav_msgs tf2_ros)
install(TARGETS scr_planar_move LIBRARY DESTINATION lib)

# Install directories
install(
  DIRECTORY urdf meshes launch rviz config worlds models
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
