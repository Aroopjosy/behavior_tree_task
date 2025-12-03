cmake_minimum_required(VERSION 3.8)
project(apple_task_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)




add_executable(talker src/cpp_publisher.cpp)
add_executable(listener src/cpp_subscriber.cpp)
add_executable(bt_sample src/bt_sample_node.cpp)
add_executable(bt_executor src/bt_executor_node.cpp)

ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(bt_sample rclcpp ament_index_cpp behaviortree_cpp )
ament_target_dependencies(bt_executor rclcpp ament_index_cpp behaviortree_cpp )



install(TARGETS
  talker
  listener
  bt_sample
  bt_executor
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.xml"
)

ament_package()
