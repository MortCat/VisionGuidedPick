cmake_minimum_required(VERSION 3.22)
project(agent_sim_servo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(moveit_servo REQUIRED)
add_executable(agent_sim_servo servo_node.cpp main.cpp)
target_link_libraries(agent_sim_servo moveit_servo::moveit_servo_lib_cpp)
ament_target_dependencies(agent_sim_servo rclcpp rclcpp_components realtime_tools
  control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
  moveit_ros_planning_interface pluginlib sensor_msgs std_msgs std_srvs tf2_eigen trajectory_msgs)
