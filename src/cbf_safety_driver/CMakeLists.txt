cmake_minimum_required(VERSION 3.16)
project(cbf_safety_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cbf_safety_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(safety_filter_core
  src/safety_filter.cpp
)
target_include_directories(safety_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(safety_filter_core PUBLIC Eigen3::Eigen)

add_executable(cbf_safety_driver_node
  src/driver_node.cpp
  src/main.cpp
)
target_link_libraries(cbf_safety_driver_node safety_filter_core)
ament_target_dependencies(cbf_safety_driver_node
  cbf_safety_interfaces
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS safety_filter_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS cbf_safety_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_filter test/test_safety_filter.cpp)
  target_link_libraries(test_safety_filter safety_filter_core)
endif()

ament_package()
