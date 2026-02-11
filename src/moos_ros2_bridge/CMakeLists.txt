cmake_minimum_required(VERSION 3.8)
project(moos_ros2_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# MOOS dependency (from MOOS-IvP build tree)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/moos-ivp/build/MOOS/MOOSCore")
find_package(MOOS REQUIRED)

# Bridge executable
add_executable(moos_ros2_bridge_node src/moos_ros2_bridge_node.cpp)
target_link_libraries(moos_ros2_bridge_node MOOS::MOOS)
ament_target_dependencies(moos_ros2_bridge_node rclcpp std_msgs)

# Install
install(TARGETS moos_ros2_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
