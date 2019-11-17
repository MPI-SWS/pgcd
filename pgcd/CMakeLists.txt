cmake_minimum_required(VERSION 3.5)
project(pgcd)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# Find ros related catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoveTo.msg"
  "msg/Ok.msg"
  "msg/Done.msg"
  "msg/Red.msg"
  "msg/Green.msg"
  "msg/MoveToStamped.msg"
  "msg/OkStamped.msg"
  "msg/DoneStamped.msg"
  "msg/RedStamped.msg"
  "msg/GreenStamped.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# python scripts
ament_python_install_package(nodes)
install(PROGRAMS nodes/component.py DESTINATION lib/${PROJECT_NAME})

# launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)

ament_package()
