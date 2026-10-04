cmake_minimum_required(VERSION 3.8)
project(vision_servo_grab_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_servo_interfaces REQUIRED)

# ─────────────────────────────────────────────────
# 控制器库（纯状态机）
# ─────────────────────────────────────────────────
add_library(grab_controller
  src/grab_controller.cpp
)
target_include_directories(grab_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(grab_controller rclcpp)

add_executable(grab_node src/grab_node.cpp)
target_link_libraries(grab_node grab_controller)
ament_target_dependencies(grab_node
  rclcpp
  std_msgs
  vision_servo_interfaces
)

install(TARGETS grab_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()