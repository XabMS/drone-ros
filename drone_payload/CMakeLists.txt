cmake_minimum_required(VERSION 3.16)
project(drone_payload CXX)

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

# Lógica de la suelta (C++ puro, IDAL C), probada sin ROS.
add_library(payload_state_machine src/payload_state_machine.cpp)
target_include_directories(payload_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(payload_state_machine PUBLIC drone_core::drone_core)
target_compile_options(payload_state_machine PRIVATE ${WARNINGS} -Wconversion)

add_executable(payload_manager src/payload_manager_node.cpp)
target_link_libraries(payload_manager payload_state_machine rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS} ${px4_msgs_TARGETS} ${drone_interfaces_TARGETS})
target_compile_options(payload_manager PRIVATE ${WARNINGS})

install(TARGETS payload_manager DESTINATION lib/${PROJECT_NAME})
install(TARGETS payload_state_machine ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(payload_state_machine_test test/payload_state_machine_test.cpp)
  target_link_libraries(payload_state_machine_test payload_state_machine)
  target_compile_options(payload_state_machine_test PRIVATE ${WARNINGS})
endif()

ament_package()
