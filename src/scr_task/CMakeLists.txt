cmake_minimum_required(VERSION 3.8)
project(scr_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(scr_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(deps
  rclcpp rclcpp_action rclcpp_lifecycle lifecycle_msgs behaviortree_cpp_v3
  nav2_behavior_tree nav2_util nav2_msgs geometry_msgs scr_interfaces ament_index_cpp)

include_directories(include)

# 任务文件解析 + 行为树节点（插件库：执行器按名字加载）
add_library(scr_task_bt_nodes SHARED src/task_files.cpp src/bt_nodes.cpp)
ament_target_dependencies(scr_task_bt_nodes ${deps})
target_link_libraries(scr_task_bt_nodes yaml-cpp)
target_compile_definitions(scr_task_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

add_executable(task_executor src/task_executor.cpp src/task_executor_main.cpp)
ament_target_dependencies(task_executor ${deps})
target_link_libraries(task_executor scr_task_bt_nodes)

install(TARGETS scr_task_bt_nodes
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS task_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 源文件不带许可证头（与仓库其他包一致），不运行 copyright 检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_files test/test_task_files.cpp
    ENV SCR_REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../..)
  target_link_libraries(test_task_files scr_task_bt_nodes)
  ament_add_gtest(test_behavior_trees test/test_behavior_trees.cpp
    ENV SCR_TASK_BT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees
    SCR_REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../..)
  target_link_libraries(test_behavior_trees scr_task_bt_nodes)
endif()

ament_export_include_directories(include)
ament_export_libraries(scr_task_bt_nodes)
ament_export_dependencies(${deps})
ament_package()
