cmake_minimum_required(VERSION 3.8)
project(ros32bot_middleware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

add_library(${PROJECT_NAME} SHARED
  src/ros32bot_interface.cpp
  src/package_bytes.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${SERIAL_LDFLAGS})
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface ros32bot_interface.xml)

# Pi-side node: cmd_vel -> UART frames, with no ros2_control involved.
add_executable(uart_bridge
  src/uart_bridge.cpp
  src/package_bytes.cpp
)
target_include_directories(uart_bridge PRIVATE
  include
  ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(uart_bridge ${SERIAL_LDFLAGS})
ament_target_dependencies(uart_bridge
  geometry_msgs
  rclcpp
)

install(TARGETS uart_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
