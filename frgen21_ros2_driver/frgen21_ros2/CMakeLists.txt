cmake_minimum_required(VERSION 3.8)
project(frgen21_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(frgen21_msgs REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS system thread regex)
find_package(frgen21_driver 1.2.0 REQUIRED)

add_executable(frgen21_node src/frgen21_driver_node.cpp
  src/frgen21_conversion.cpp
  src/frgen21_driver.cpp
)
ament_target_dependencies(frgen21_node
  rclcpp
  std_msgs
  sensor_msgs
  frgen21_msgs
)

target_link_libraries(frgen21_node frgen21_driver::frgen21_driver ${Boost_LIBRARIES} ${Boost_DEPENDENCIES})
target_include_directories(frgen21_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${frgen21_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(frgen21_service src/frgen21_service.cpp
)
ament_target_dependencies(frgen21_service 
  rclcpp 
  frgen21_msgs)
target_link_libraries(frgen21_service frgen21_driver::frgen21_driver)
target_include_directories(frgen21_service PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${frgen21_INCLUDE_DIRS})

install(TARGETS
frgen21_node
frgen21_service
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  rviz
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
