cmake_minimum_required(VERSION 3.16)
project(mocap_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(mocap_natnet STATIC
  third_party/MoCapParser/NatNetClient.cpp
)
target_include_directories(mocap_natnet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
)
target_link_libraries(mocap_natnet PUBLIC Eigen3::Eigen PRIVATE Threads::Threads)

add_executable(mocap_bridge_node src/mocap_bridge_node.cpp)
target_link_libraries(mocap_bridge_node PRIVATE
  mocap_natnet
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

install(TARGETS mocap_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
