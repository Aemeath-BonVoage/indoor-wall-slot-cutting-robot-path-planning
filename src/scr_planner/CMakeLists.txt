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

# ROS-free planner core + C API (loaded from Python via ctypes)
add_library(scr_planner_core SHARED src/oriented_astar.cpp src/c_api.cpp)
target_include_directories(scr_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS scr_planner_core EXPORT export_scr_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES python/scr_planner_py.py python/wall_rules.py python/slot_plan.py python/slot_ros.py
  DESTINATION share/${PROJECT_NAME}/python)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/wall_slot_planner_node.py scripts/execute_slot_plan.py
  scripts/plan_wall_slots.py scripts/plot_execution.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_oriented_astar test/test_oriented_astar.cpp)
  target_link_libraries(test_oriented_astar scr_planner_core)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_slot_plan test/test_slot_plan.py
    ENV SCR_PLANNER_LIB=$<TARGET_FILE:scr_planner_core>
    TIMEOUT 300)
endif()

ament_export_include_directories(include)
ament_export_targets(export_scr_planner HAS_LIBRARY_TARGET)
ament_package()
