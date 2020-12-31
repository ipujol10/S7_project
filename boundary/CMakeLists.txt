cmake_minimum_required(VERSION 3.0.2)
project(boundary)


find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs geometry_msgs sensor_msgs message_generation tf)

#catkin_package()
#catkin_python_setup()



catkin_package(
#  INCLUDE_DIRS include
#   LIBRARIES webots_ros
   CATKIN_DEPENDS rospy 
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS src/boundary.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
