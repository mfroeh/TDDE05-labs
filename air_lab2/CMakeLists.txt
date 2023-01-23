cmake_minimum_required(VERSION 3.8)
project(air_lab2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(lab2_node src/lab2_node.cpp)
target_include_directories(lab2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lab2_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(lab2_node rclcpp std_msgs geometry_msgs nav_msgs rclcpp_action rclcpp_components)
install(TARGETS lab2_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(random_exploration src/random_exploration.cpp)
target_include_directories(random_exploration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(random_exploration PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(random_exploration rclcpp std_msgs geometry_msgs nav_msgs rclcpp_action rclcpp_components)

install(TARGETS random_exploration
  DESTINATION lib/${PROJECT_NAME})

ament_package()
