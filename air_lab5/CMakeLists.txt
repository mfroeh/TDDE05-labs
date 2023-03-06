cmake_minimum_required(VERSION 3.8)
project(air_lab5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(air_lab_interfaces REQUIRED)

add_executable(text_command src/text_command.cpp)
target_include_directories(text_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(text_command PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(text_command rclcpp std_msgs air_lab_interfaces)
install(TARGETS text_command
  DESTINATION lib/${PROJECT_NAME})

SET(CMAKE_EXPORT_COMPILE_COMMANDS True)

add_executable(decision src/decision.cpp)
target_include_directories(decision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(decision PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(decision rclcpp std_msgs)

install(TARGETS decision
  DESTINATION lib/${PROJECT_NAME})

ament_package()

