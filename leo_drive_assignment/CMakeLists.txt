cmake_minimum_required(VERSION 3.8)
project(leo_drive_assignment)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_PREFIX_PATH "/usr/local")
#set(geographic_msgs_DIR "/home/melike/geographic_msgs/install/share/geographic_msgs/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


#include_directories(${LIBKML_INCLUDE_DIRS})
#target_link_libraries(point_cloud ${LIBKML_LIBRARIES})


#ament_target_dependencies(${PROJECT_NAME} "pcl_ros")
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_publisher src/pcl.cpp)
target_link_libraries( point_cloud_publisher
  ${PCL_LIBRARIES}
)
add_executable(kml_parser_node src/kml_parser_node.cpp)

ament_target_dependencies(point_cloud_publisher
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)
ament_target_dependencies(kml_parser_node 
  rclcpp 
  geographic_msgs
  nav_msgs
  geometry_msgs
  GeographicLib
  
  )
target_link_libraries(kml_parser_node GeographicLib::GeographicLib)


install(TARGETS point_cloud_publisher
  DESTINATION lib/leo_drive_assignment
)
install(TARGETS kml_parser_node
  DESTINATION lib/leo_drive_assignment
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
