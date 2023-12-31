cmake_minimum_required(VERSION 3.8)
project(eddiebot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(eddiebot_msgs REQUIRED)

add_executable(eddie src/eddie.cpp)
target_include_directories(eddie PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(eddie PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(eddie rclcpp eddiebot_msgs)

add_executable(eddie_controller src/eddie_controller.cpp)
target_include_directories(eddie_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(eddie_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(eddie_controller rclcpp eddiebot_msgs)

add_executable(eddie_adc src/eddie_adc.cpp)
target_include_directories(eddie_adc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(eddie_adc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(eddie_adc rclcpp eddiebot_msgs)

add_executable(eddie_ping src/eddie_ping.cpp)
target_include_directories(eddie_ping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(eddie_ping PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(eddie_ping rclcpp eddiebot_msgs)

install(TARGETS
  eddie
  eddie_controller
  eddie_adc
  eddie_ping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
