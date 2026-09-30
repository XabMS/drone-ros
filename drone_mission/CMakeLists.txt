cmake_minimum_required(VERSION 3.16)
project(drone_mission CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(drone_core REQUIRED)

set(WARNINGS -Wall -Wextra -Wpedantic -Werror -Wshadow)

# Lógica de misión (C++ puro, IDAL C), probada sin ROS.
add_library(mission_state_machine src/mission_state_machine.cpp)
target_include_directories(mission_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_state_machine PUBLIC drone_core::drone_core)
target_compile_options(mission_state_machine PRIVATE ${WARNINGS} -Wconversion)

add_executable(config_manager src/config_manager_node.cpp)
target_link_libraries(config_manager drone_core::drone_core rclcpp::rclcpp
  ${std_srvs_TARGETS} ${drone_interfaces_TARGETS})
target_compile_options(config_manager PRIVATE ${WARNINGS})

add_executable(mission_manager src/mission_manager_node.cpp)
target_link_libraries(mission_manager mission_state_machine rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${px4_msgs_TARGETS} ${drone_interfaces_TARGETS})
target_compile_options(mission_manager PRIVATE ${WARNINGS})

install(TARGETS config_manager mission_manager DESTINATION lib/${PROJECT_NAME})
install(TARGETS mission_state_machine ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mission_state_machine_test test/mission_state_machine_test.cpp)
  target_link_libraries(mission_state_machine_test mission_state_machine)
endif()

ament_package()
