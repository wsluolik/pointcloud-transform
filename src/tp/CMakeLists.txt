cmake_minimum_required(VERSION 2.8.3)
project(tp)
SET( CMAKE_BUILD_TYPE "Release" )
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS
              cv_bridge
              genmsg
              pcl_conversions
              pcl_ros
              roscpp
              rospy
              std_msgs
	      geometry_msgs
              message_generation
              )


## find PCL
find_package(PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

add_message_files(
		FILES

)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
)

catkin_package(
		CATKIN_DEPENDS  std_msgs geometry_msgs message_runtime
)

add_definitions( ${EIGEN_DEFINITIONS} )
add_definitions( ${PCL_DEFINITIONS} )

include_directories(include ${catkin_INCLUDE_DIRS})

# test receive perception results
add_executable( tp
		src/tp.cpp
		)
target_link_libraries(tp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
