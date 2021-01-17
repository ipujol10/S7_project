cmake_minimum_required(VERSION 3.0.2)
project(simple_alg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
	CATKIN_DEPENDS
		geometry_msgs
  		roscpp
  		rospy
  		std_msgs
  	)

include_directories(

  ${catkin_INCLUDE_DIRS}
)


add_executable(simple_alg src/algorithm1.cpp) ##VERY IMPORTANT
add_executable(test_sub src/test_subscriber.cpp) ##VERY IMPORTANT
add_executable(test_pub src/test_publisher.cpp) ##VERY IMPORTANT


add_dependencies(simple_alg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(simple_alg ${catkin_LIBRARIES})
target_link_libraries(test_sub ${catkin_LIBRARIES})
target_link_libraries(test_pub ${catkin_LIBRARIES})


