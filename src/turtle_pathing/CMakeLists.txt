cmake_minimum_required(VERSION 3.8)
project(turtle_pathing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_interfaces REQUIRED)

include_directories(include)

add_executable(turtle_controller src/turtle_controller.cpp)
add_executable(turtle_spawner src/turtle_spawner.cpp)

ament_target_dependencies(turtle_controller rclcpp geometry_msgs turtlesim turtle_interfaces)
ament_target_dependencies(turtle_spawner rclcpp geometry_msgs turtlesim turtle_interfaces)

install(TARGETS
  turtle_controller
  turtle_spawner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

ament_package()
