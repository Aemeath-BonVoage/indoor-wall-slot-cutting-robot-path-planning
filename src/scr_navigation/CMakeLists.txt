cmake_minimum_required(VERSION 3.5)
project(scr_navigation)

find_package(ament_cmake REQUIRED)

# 安装配置文件
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/set_initial_pose.py
  scripts/send_goal.py
  scripts/nav_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
