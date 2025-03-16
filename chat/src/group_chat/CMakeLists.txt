cmake_minimum_required(VERSION 3.8)
project(group_chat)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(chat_node_1 src/chat_node_1.cpp)
ament_target_dependencies(chat_node_1 rclcpp std_msgs)

add_executable(chat_node_2 src/chat_node_2.cpp)
ament_target_dependencies(chat_node_2 rclcpp std_msgs)
add_executable(chat_node src/chat_node.cpp)
ament_target_dependencies(chat_node rclcpp std_msgs)


install(TARGETS
  chat_node_1
  chat_node_2
  chat_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

