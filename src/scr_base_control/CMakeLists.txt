cmake_minimum_required(VERSION 3.8)
project(scr_base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

include_directories(include)

# 控制器插件（controller_manager 加载）
add_library(scr_base_control SHARED src/mecanum_drive_controller.cpp)
ament_target_dependencies(scr_base_control
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools
  geometry_msgs nav_msgs tf2_msgs)
pluginlib_export_plugin_description_file(controller_interface scr_base_control_plugins.xml)

# 仿真硬件接口插件（gazebo_ros2_control 加载）
add_library(scr_base_control_gazebo SHARED src/mecanum_base_system.cpp)
ament_target_dependencies(scr_base_control_gazebo
  gazebo_dev gazebo_ros gazebo_ros2_control hardware_interface pluginlib rclcpp)
pluginlib_export_plugin_description_file(gazebo_ros2_control scr_base_control_gazebo.xml)

install(TARGETS scr_base_control scr_base_control_gazebo
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/base_characterization.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 源文件不带许可证头（与仓库其他包一致），不运行 copyright 检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mecanum_kinematics test/test_mecanum_kinematics.cpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(scr_base_control)
ament_package()
