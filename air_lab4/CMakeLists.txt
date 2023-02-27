cmake_minimum_required(VERSION 3.8)
project(air_lab4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros2_kdb_msgs REQUIRED)
find_package(TstML REQUIRED)

add_executable(visualizer src/visualizer.cpp)
target_include_directories(visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(visualizer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
    visualizer
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    ros2_kdb_msgs
    TstML
)
target_link_libraries(visualizer TstMLExecutor)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

install(TARGETS visualizer
  DESTINATION lib/${PROJECT_NAME})
ament_package()
