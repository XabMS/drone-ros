cmake_minimum_required(VERSION 3.16)
project(drone_core CXX)

# drone_core: librería C++ común sin dependencias de ROS (geodesia, geometría,
# configuración por ciudad, misiones). Se compila con ament si está disponible
# y con CMake puro si no (para probarla fuera del workspace).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(DRONE_CORE_COVERAGE "Instrumentar para cobertura" OFF)

find_package(ament_cmake QUIET)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(drone_core
  src/geo.cpp
  src/geometry.cpp
  src/ops_config.cpp
  src/mission.cpp
)
target_include_directories(drone_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(drone_core PUBLIC yaml-cpp nlohmann_json::nlohmann_json)
target_compile_options(drone_core PRIVATE -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion)
set_target_properties(drone_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(DRONE_CORE_COVERAGE)
  target_compile_options(drone_core PRIVATE --coverage -O0)
  target_link_options(drone_core PUBLIC --coverage)
endif()

add_executable(drone_validate tools/drone_validate.cpp)
target_link_libraries(drone_validate drone_core)
target_compile_options(drone_validate PRIVATE -Wall -Wextra -Werror)

if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS drone_validate DESTINATION lib/${PROJECT_NAME})
  install(TARGETS drone_core EXPORT export_drone_core
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  ament_export_targets(export_drone_core HAS_LIBRARY_TARGET)
  ament_export_dependencies(yaml-cpp nlohmann_json)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(drone_core_test test/drone_core_test.cpp
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(drone_core_test drone_core)
  endif()
  ament_package()
else()
  find_package(GTest REQUIRED)
  enable_testing()
  add_executable(drone_core_test test/drone_core_test.cpp)
  target_link_libraries(drone_core_test drone_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(drone_core_test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()
