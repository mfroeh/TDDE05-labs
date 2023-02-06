cmake_minimum_required(VERSION 3.8)
project(air_lab3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(air_lab_interfaces REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(TstML REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(tst_executor src/tst_executor.cpp)
target_include_directories(tst_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tst_executor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  tst_executor
  "rclcpp"
  "rclcpp_action"
  "air_lab_interfaces"
  "irobot_create_msgs"
  "TstML"
  std_srvs
  "nav2_msgs"
)
target_link_libraries(tst_executor TstMLExecutor)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

install(TARGETS tst_executor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
