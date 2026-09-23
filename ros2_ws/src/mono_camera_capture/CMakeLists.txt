cmake_minimum_required(VERSION 3.8)
project(mono_camera_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

add_library(${PROJECT_NAME}_lib
  src/mono_camera_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
)

add_executable(mono_camera_node
  src/mono_camera_node_main.cpp
)

target_link_libraries(mono_camera_node
  ${PROJECT_NAME}_lib
)

install(TARGETS
  ${PROJECT_NAME}_lib
  mono_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  cv_bridge
)

ament_package()
