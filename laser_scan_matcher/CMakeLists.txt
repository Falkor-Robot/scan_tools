cmake_minimum_required(VERSION 3.5)
project(laser_scan_matcher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

## Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()

# Set minimum C++ standard to C++14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  rclcpp
#  nodelet
  sensor_msgs
  tf2
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs)

# Find catkin and all required ROS components
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(generate_parameter_library REQUIRED)

#find_package(ament_cmake REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(PCL REQUIRED QUIET)


# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})


generate_parameter_library(laser_scan_matcher_parameters
  src/laser_scan_matcher_parameters.yaml
)

# Declare info that other packages need to import library generated here
#catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES laser_scan_matcher
#    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
#)

#Create library
add_library(laser_scan_matcher src/laser_scan_matcher.cpp)

target_link_libraries(laser_scan_matcher PUBLIC
laser_scan_matcher_parameters
csm
)

#Note we don't link against pcl as we're using header-only parts of the library
#target_link_libraries( laser_scan_matcher ${catkin_LIBRARIES} ${csm_LIBRARIES})
ament_target_dependencies( laser_scan_matcher 
        PUBLIC
        csm
        rclcpp
        tf2
        tf2_ros
        pcl_ros
        pcl_conversions
        sensor_msgs
        geometry_msgs
        nav_msgs
        )

#Create nodelet
#add_library(laser_scan_matcher_nodelet src/laser_scan_matcher_nodelet.cpp)
#target_link_libraries(laser_scan_matcher_nodelet laser_scan_matcher)

#Create node
add_executable(laser_scan_matcher_node src/laser_scan_matcher_node.cpp)
target_link_libraries( laser_scan_matcher_node laser_scan_matcher )

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

#Install library includes
install(
    DIRECTORY include/laser_scan_matcher/
    DESTINATION include
)
ament_export_include_directories(include)

#Install node
install(TARGETS laser_scan_matcher_node
    DESTINATION lib/${PROJECT_NAME} )


#Install demo files
install(DIRECTORY demo
    DESTINATION share/${PROJECT_NAME} )

if(BUILD_TESTING)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    
    #add_rostest(test/covariance.test)
    #add_rostest(test/run.test)
endif()

ament_package()