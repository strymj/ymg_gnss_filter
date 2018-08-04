cmake_minimum_required(VERSION 2.8.3)
project(ymg_gnss_filter)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
	tf2
	nmea_navsat_driver
	nmea_msgs
)

catkin_package(
	INCLUDE_DIRS include
	# LIBRARIES mylib
	# CATKIN_DEPENDS roscpp rospy
	# DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

add_library(gnss_filter
	src/gnss_filter.cpp
	)

add_executable(gnss_filter_node
	main/gnss_filter_node.cpp
	)
target_link_libraries(gnss_filter_node
	gnss_filter
	${catkin_LIBRARIES}
	)	
