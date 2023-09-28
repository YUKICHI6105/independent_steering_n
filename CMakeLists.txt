cmake_minimum_required(VERSION 3.8)
project(independent_steering_n)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic-errors -std=c++20 -latomic -fmax-errors=3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_plugins2 REQUIRED)
# find_package(fake_robomaster_serial_can REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

set(msg_files
  "msg/LinearVelocity.msg"
  "msg/AngularVelocity.msg"
  "msg/TxFloat.msg"
)

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

add_executable(node
  src/node.cpp
)

add_executable(tx_listener
  src/tx_listener.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(node "${cpp_typesupport_target}" atomic)
target_link_libraries(tx_listener "${cpp_typesupport_target}")

# add_library(node SHARED
#   src/independent_steering_n.cpp
# )
# rclcpp_components_register_nodes(node "nhk2024::independent_steering_n::Node")

ament_target_dependencies(node
  rclcpp
  rclcpp_components
  std_msgs
  can_plugins2
  # fake_robomaster_serial_can
)

ament_target_dependencies(tx_listener
  rclcpp
  rclcpp_components
  can_plugins2
)

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS node
  TARGETS node tx_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
