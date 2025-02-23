cmake_minimum_required(VERSION 3.8)
project(chase_object)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(chase_object_pubsub src/chase_object_pubsub.cpp)
add_executable(test_publisher src/test_publisher.cpp)
ament_target_dependencies(chase_object_pubsub rclcpp geometry_msgs)
ament_target_dependencies(test_publisher rclcpp geometry_msgs)

# Install the executable
install(TARGETS chase_object_pubsub DESTINATION lib/chase_object)
install(TARGETS
  test_publisher
  DESTINATION lib/chase_object)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
