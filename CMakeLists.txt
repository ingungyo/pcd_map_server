cmake_minimum_required(VERSION 3.8)
project(pcd_map_server)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# set(PCD_COMPONENT_LIB pcd_map_server_component)

add_library(${PROJECT_NAME} SHARED
  src/pcd_map_server.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  # rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

add_executable(pcd_map_server_node src/pcd_map_server_node.cpp)
target_link_libraries(pcd_map_server_node ${PROJECT_NAME}) 
ament_target_dependencies(pcd_map_server_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  pcl_conversions
)

install(TARGETS
  ${PROJECT_NAME}   
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  pcd_map_server_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(FILES resource/pcd_map_server_plugins.xml
  DESTINATION share/${PROJECT_NAME})


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
