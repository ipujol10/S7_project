#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <iostream>
#include <random>
#include <math.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"


double imu_x = 1;
double imu_y = 2;
double imu_z = 3;
double gps_x = 4;
double gps_y = 5;
double gps_z = 6;
bool boundary_stern = false;
bool boundary_keel = false;
bool boundary_bow = false;


int main(int argc, char **argv){
	
	ros::init(argc, argv, "TEST_PUBLISHER"); //initialise ROS topic for publishing and subscribing
	ros::NodeHandle n; //initialise communication
	
	//publishing to the local planner the goal position
    	ros::Publisher robot_orient_pub = n.advertise<geometry_msgs::Vector3>("robot_orient", 1000);
    	ros::Publisher robot_pose_pub = n.advertise<geometry_msgs::Vector3>("robot_pose", 1000);
    	ros::Publisher boundary_stern_pub = n.advertise<std_msgs::Bool>("boundary_stern", 1000);	
    	ros::Publisher boundary_keel_pub = n.advertise<std_msgs::Bool>("boundary_keel", 1000);	
    	ros::Publisher boundary_bow_pub = n.advertise<std_msgs::Bool>("boundary_bow", 1000);	
	
	ros::Rate loop_rate(10); //frequency setup, here it runs at 10Hz


	while (ros::ok()){
		
	geometry_msgs::Vector3 robot_orient = geometry_msgs::Vector3();
	geometry_msgs::Vector3 robot_pose = geometry_msgs::Vector3();
	std_msgs::Bool boundary_stern_bool = std_msgs::Bool();
	std_msgs::Bool boundary_keel_bool = std_msgs::Bool();
	std_msgs::Bool boundary_bow_bool = std_msgs::Bool();
		
    	robot_orient.x = imu_x;
    	robot_orient.y = imu_y;
    	robot_orient.z = imu_z;
    	
    	robot_pose.x = gps_x;
    	robot_pose.y = gps_y;
    	robot_pose.z = gps_z;
    	
    	boundary_stern_bool.data = true;
    	boundary_keel_bool.data = false;
    	boundary_bow_bool.data = true;
    		
    	robot_orient_pub.publish(robot_orient); //here we publish our message
    	robot_pose_pub.publish(robot_pose); //here we publish our message
    	boundary_stern_pub.publish(boundary_stern_bool); //here we publish our message
    	boundary_keel_pub.publish(boundary_keel_bool); //here we publish our message
    	boundary_bow_pub.publish(boundary_bow_bool); //here we publish our message
		
	ros::spinOnce(); //neccessary for a later subscription
	
	loop_rate.sleep(); //sleep until it hits 10Hz publish rate
	}
	
	
	return 0;

}


