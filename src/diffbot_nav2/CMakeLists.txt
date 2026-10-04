cmake_minimum_required(VERSION 3.8)
project(diffbot_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(robot_localization REQUIRED)

# 安装静态与脚本目录至 install/share/diffbot_nav2
install(
  DIRECTORY urdf launch config maps world rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行任务脚本至 install/lib/diffbot_nav2 (供 ros2 run 调用)
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

ament_package()