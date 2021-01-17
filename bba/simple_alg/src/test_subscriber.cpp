#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

double goal_x = 0;
double goal_y = 0;
double goal_z = 0;

void goal_updater(const geometry_msgs::Vector3::ConstPtr& goal_position){
	
	goal_x = goal_position->x;
    goal_y = goal_position->y;
    goal_z = goal_position->z;
    
  	ROS_INFO("I heard: [%f], [%f], [%f]", goal_x, goal_y, goal_z);
}

int main(int argc, char **argv){
  
    ros::init(argc, argv, "listener");

 
    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe("goal", 1000, goal_updater);

  
    ros::spin();

    return 0;
}




