cmake_minimum_required(VERSION 3.8)
project(lowpass_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)



add_executable(main src/main.cpp src/lowpass_filter.cpp src/lowpass_filter_node.cpp src/config_loader.cpp)
ament_target_dependencies(main rclcpp sensor_msgs nav_msgs std_msgs)
target_include_directories(main PUBLIC include)
target_link_libraries(main yaml-cpp)

add_executable(test_single_process src/test_single_process.cpp src/lowpass_filter.cpp src/lowpass_filter_node.cpp src/config_loader.cpp)
ament_target_dependencies(test_single_process rclcpp sensor_msgs nav_msgs std_msgs)
target_include_directories(test_single_process PUBLIC include)
target_link_libraries(test_single_process yaml-cpp)

add_executable(test_publish_performance src/filter_perf_test.cpp src/lowpass_filter.cpp)
ament_target_dependencies(test_publish_performance rclcpp sensor_msgs)
target_include_directories(test_publish_performance PUBLIC include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  main
  test_single_process
  test_publish_performance
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
