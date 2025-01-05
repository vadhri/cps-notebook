cmake_minimum_required(VERSION 3.8)
project(topic_subscriber_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter checks if needed
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create executable for simple subscriber
add_executable(simple_subscriber_node src/simple_topic_subscriber.cpp)
ament_target_dependencies(simple_subscriber_node rclcpp std_msgs)

# Install simple_subscriber_node
install(TARGETS
   simple_subscriber_node
   DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Create executable for sub_odom_node
add_executable(sub_odom_node src/sub_odom.cpp)

# Add dependencies for sub_odom_node
ament_target_dependencies(sub_odom_node rclcpp std_msgs nav_msgs rcutils rcl_interfaces)

# Install sub_odom_node
install(TARGETS
   sub_odom_node
   DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
