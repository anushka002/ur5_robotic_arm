cmake_minimum_required(VERSION 3.8)
project(ur5_description)
set(CMAKE_CXX_STANDARD 14)

# if compilers are gnu or clang, then enable more compile information
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add the executable target for your C++ node
add_executable(robot_movement_node src/robot_movement_node.cpp)

# Link the executable to the required ROS 2 libraries
ament_target_dependencies(robot_movement_node rclcpp trajectory_msgs)

# install required files to share destination
install(
    DIRECTORY config include launch meshes rviz urdf worlds 
    DESTINATION share/${PROJECT_NAME}/
)

# Install your executables
install(TARGETS
  robot_movement_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament operations
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
