cmake_minimum_required(VERSION 3.8)
project(scr_planner)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util nav_msgs
  geometry_msgs tf2 tf2_ros tf2_geometry_msgs pluginlib)

# ROS-free search core (unit tests + offline benchmark)
add_library(scr_planner_core SHARED src/oriented_astar.cpp)
target_include_directories(scr_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Nav2 plugin
add_library(scr_planner SHARED src/enhanced_astar_planner.cpp)
target_include_directories(scr_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scr_planner scr_planner_core)
ament_target_dependencies(scr_planner ${dependencies})
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# Offline benchmark on a plain grid file
add_executable(bench_grid benchmark/bench_grid.cpp)
target_link_libraries(bench_grid scr_planner_core)

install(TARGETS scr_planner_core scr_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS bench_grid RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(FILES global_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/compare_with_python.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_oriented_astar test/test_oriented_astar.cpp)
  target_link_libraries(test_oriented_astar scr_planner_core)

  ament_add_gtest(test_planner_plugin test/test_planner_plugin.cpp)
  target_link_libraries(test_planner_plugin scr_planner)
  ament_target_dependencies(test_planner_plugin ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(scr_planner_core scr_planner)
ament_export_dependencies(${dependencies})
ament_package()
