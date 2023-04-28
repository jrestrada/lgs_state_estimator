cmake_minimum_required(VERSION 3.8)
project(lgs_state_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(kalman_filter 
src/kalman_filter.cpp
)

include_directories(${EIGEN3_INCLUDE_DIRS})
target_include_directories(kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kalman_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(kalman_filter
  rclcpp
  std_msgs
  Eigen3
)

install(TARGETS kalman_filter
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
