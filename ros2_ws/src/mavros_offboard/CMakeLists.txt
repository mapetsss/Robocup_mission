cmake_minimum_required(VERSION 3.8)
project(mavros_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mavros_offboard_node src/mavros_offboard_node.cpp)
add_executable(robocup_v1 src/robocup_v1.cpp)

target_compile_features(mavros_offboard_node PUBLIC cxx_std_17)
target_compile_features(robocup_v1 PUBLIC cxx_std_17)

ament_target_dependencies(mavros_offboard_node
  rclcpp
  mavros_msgs
  geometry_msgs
  std_srvs
)
ament_target_dependencies(robocup_v1
  rclcpp
  mavros_msgs
  geometry_msgs
  std_srvs
)

install(TARGETS
  mavros_offboard_node
  robocup_v1
  DESTINATION lib/${PROJECT_NAME})

# ========== 关键：安装 launch 文件 ==========
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
