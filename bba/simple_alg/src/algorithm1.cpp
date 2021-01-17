#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <iostream>
#include <random>
#include <math.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Quaternion.h"
#include "unistd.h"

double imu_x;
double imu_y;
double imu_z;

double gps_x;
double gps_y;
double gps_z;
double t;

double norm_x;
double norm_y;
double norm_z;

double goal_x;
double goal_y;
double goal_z;

double column1_x;
double column1_y;
double column1_z;
double column2_x;
double column2_y;
double column2_z;
double column3_x;
double column3_y;
double column3_z;

double full_time = 3600*6;

int forward_constant = 1;
int backward_constant = -1;
/*good ones*/
int top_boundary = 25;
int bottom_boundary =-12;
int left_boundary = 53;
int right_boundary = -45;
/*for testing*/
/*
int bottom_boundary = -5;
int left_boundary = 5;
int right_boundary = -5;
int top_boundary = 5;
*/

std_msgs::Bool finish_bool;
bool finish =false;

std_msgs::Bool reset_bool;
bool reset = false;


bool boundary;
bool reached_goal;
bool goingSterntoBow;
bool doneWithSide=false;
bool finito;
bool visitedLeft = false;
bool visitedRight = false;

geometry_msgs::Vector3 goalposition;

// below is setup for the functions that listen if any messages were published

//reset function
void resetting(ros::Publisher goalposition_pub, ros::Publisher reset_pub, ros::Rate loop_rate){
	reset = false;
	reset_pub.publish(reset_bool);
	goalposition.x = gps_x;
	goalposition.y = gps_y;
	goalposition.z = gps_z;
	goalposition_pub.publish(goalposition);

	ros::spinOnce(); //neccessary for a later subscription
	loop_rate.sleep(); //sleep until it hits 10Hz publish rate
}

//get IMU data
void imu_updater(const geometry_msgs::Vector3::ConstPtr& robot_orient){

	imu_x = floor(robot_orient->x); //pitch
	imu_y = floor(robot_orient->y); //yaw
	imu_z = floor(robot_orient->z); //roll

	//ROS_INFO("Received New IMU readings @ [%f],[%f],[%f]", imu_x, imu_y, imu_z);
}


//get GPS pose from the beacons
void gps_updater(const geometry_msgs::Quaternion::ConstPtr& robot_pose){

	gps_x = robot_pose->x;
	gps_y = robot_pose->y;
	gps_z = robot_pose->z;
	t = robot_pose->w;

	//ROS_INFO("Received New GPS Position @ [%f],[%f],[%f]", gps_x, gps_y, gps_z);
}

void goal_reached(const std_msgs::Bool::ConstPtr& goal_reached_bool){
	reached_goal = goal_reached_bool->data;
}
	/*if (boundary_stern == false){
		ROS_INFO("Boundary on the stern not reached yet.");
	}

}*/


void matrix_1(const geometry_msgs::Vector3::ConstPtr& column_1){

	column1_x = floor(column_1->x);
	column1_y = floor(column_1->y);
	column1_z = floor(column_1->z);

	//ROS_INFO("Received New Column1 readings @ [%f],[%f],[%f]", column1_x, column1_y, column1_z);
}


void matrix_2(const geometry_msgs::Vector3::ConstPtr& column_2){

	column2_x = floor(column_2->x);
	column2_y = floor(column_2->y);
	column2_z = floor(column_2->z);

	//ROS_INFO("Received New Column2 readings @ [%f],[%f],[%f]", column2_x, column2_y, column2_z);
}


void matrix_3(const geometry_msgs::Vector3::ConstPtr& column_3){

	column3_x = floor(column_3->x);
	column3_y = floor(column_3->y);
	column3_z = floor(column_3->z);

	//ROS_INFO("Received New Column3 readings @ [%f],[%f],[%f]", column3_x, column3_y, column3_z);
}


//here the algorithm is defined, starts with going to the water's surface and then turns to the bow's boundary
double start_up(double robot_pose_gps_reading, double movement_constant){

	double goal_pose = robot_pose_gps_reading + movement_constant;

	return goal_pose;
}

double goDownTheBoundary(bool boundary,ros::Publisher goalposition_pub, ros::Rate loop_rate){

	ROS_INFO("boundary");

	if (goal_x >= left_boundary || goal_x <= right_boundary){
		ROS_INFO("going down");
		goal_y = goal_y + backward_constant;

		if (goal_x >= left_boundary && goal_y >= bottom_boundary){
			ROS_INFO("Last path to the right");
			visitedLeft = true;
		}
		if (goal_x <= right_boundary && goal_y >= bottom_boundary){
			ROS_INFO("Last path to the left");
			visitedRight = true;
		}

		if (visitedLeft && visitedRight ){
			doneWithSide = true;
		}

	}


	ROS_INFO("boundary [%f],[%f],[%f]",gps_x,gps_y,gps_z);
	ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
	goalposition.x = gps_x;
	goalposition.y = goal_y;
	goalposition.z = gps_z;

	goalposition_pub.publish(goalposition);


	goingSterntoBow = !goingSterntoBow;
	boundary = false;

	ros::spinOnce(); //neccessary for a later subscription
	loop_rate.sleep(); //sleep until it hits 10Hz publish rate

	return goal_y;
}


//clean the boat
void crossBoat(bool goingSterntoBow,ros::Publisher goalposition_pub,ros::Publisher finish_pub, ros::Rate loop_rate){
	ROS_INFO("cleaning");
	while (gps_x <= left_boundary  || gps_x >= right_boundary){

		if (goal_x >= left_boundary || goal_x <= right_boundary){
			goDownTheBoundary(true, goalposition_pub, loop_rate);
		}

		if (goingSterntoBow){
			ROS_INFO("going right");
			ROS_INFO("time: [%f]",t);
			ROS_INFO("full_time;[%f]",full_time);
			goal_x = gps_x + backward_constant;
			ROS_INFO("right [%f],[%f],[%f]",gps_x,gps_y,gps_z);
			ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
			if (full_time < t){
				bool finish=true;
				finish_bool.data = finish;
				finish_pub.publish(finish_bool);
			}
			if (goal_x <= right_boundary){
				goingSterntoBow = false;
			} //We changed the test to be larger or equal
		}
		else {
		ROS_INFO("going left");
		ROS_INFO("time: [%f]",t);
		ROS_INFO("full_time;[%f]",full_time);
		goal_x = gps_x + forward_constant;
		ROS_INFO("left [%f],[%f],[%f]",gps_x,gps_y,gps_z);
		ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
		if (t >= full_time){
			bool finish=true;
			finish_bool.data = finish;
			finish_pub.publish(finish_bool);
		 }
		 if (goal_x >= left_boundary){
			 goingSterntoBow = true;
		 }
	 }


		if (visitedLeft){

			/*if (gps_x <= left_boundary){
				ROS_INFO("last going left");
				goal_x = gps_x + forward_constant;
				ROS_INFO("left [%f],[%f],[%f]",gps_x,gps_y,gps_z);
				ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
			}*/

			if (goal_x <= right_boundary){
				ROS_INFO("last going right");
				gps_x = gps_x + backward_constant;
				ROS_INFO("right [%f],[%f],[%f]",gps_x,gps_y,gps_z);
				ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
				if (t >= full_time){
					bool finish=true;
					finish_bool.data = finish;
					finish_pub.publish(finish_bool);
				}
			}
		}
			if (visitedRight){

				if (gps_x >= left_boundary){
					ROS_INFO("last going left");
					goal_x = gps_x + forward_constant;
					ROS_INFO("left [%f],[%f],[%f]",gps_x,gps_y,gps_z);
					ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
					if (t >= full_time){
						bool finish=true;
						finish_bool.data = finish;
						finish_pub.publish(finish_bool);
					}
				}
				/*else if (goal_x >= right_boundary){
					ROS_INFO("last going right");
					gps_x = gps_x + backward_constant;
					ROS_INFO("right [%f],[%f],[%f]",gps_x,gps_y,gps_z);
					ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
				}*/
			}
		// go to the other side of boat			+


		while (doneWithSide){
			ROS_INFO("goodnight");
			ROS_INFO("position[%f],[%f],[%f]",gps_x,gps_y,gps_z);
			ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
			bool finish=true;
			finish_bool.data = finish;
			goalposition.x = goal_x;
			goalposition.y = goal_y;
			goalposition.z = gps_z;

			goalposition_pub.publish(goalposition);
			finish_pub.publish(finish_bool);
			ros::spinOnce(); //neccessary for a later subscription

			loop_rate.sleep(); //sleep until it hits 10Hz publish rate


		}



		//go down the boundary
		goalposition.x = goal_x;
    goalposition.y = goal_y;
    goalposition.z = gps_z;

		goalposition_pub.publish(goalposition);
		finish_pub.publish(finish_bool);
		ros::spinOnce(); //neccessary for a later subscription

		loop_rate.sleep(); //sleep until it hits 10Hz publish rate


	}


}

void path_plan(ros::Publisher goalposition_pub,ros::Publisher finish_pub, ros::Rate loop_rate){


	bool visitedLeft = false;
	bool visitedRight = false;
	bool goingSterntoBow = true;
	bool doneWithSide = false;
	bool boundary = true;
	//go straight to the goal
	while (!doneWithSide){
	//	ROS_INFO("wonky donky");
		crossBoat(true, goalposition_pub, finish_pub, loop_rate);
    goingSterntoBow = !goingSterntoBow;
}
		ros::spinOnce(); //neccessary for a later subscription

		loop_rate.sleep(); //sleep until it hits 10Hz publish rate

}

int main(int argc, char **argv){

//ROS_INFO("here");
	ros::init(argc, argv, "MY_GLOBAL_PLANNER"); //initialise ROS topic for publishing and subscribing
	ros::NodeHandle n; //initialise communication

	//publishing to the local planner the goal position
  ros::Publisher goalposition_pub = n.advertise<geometry_msgs::Vector3>("goal", 1000);
	ros::Publisher  finish_pub= n.advertise<std_msgs::Bool>("finish", 1000);
	ros::Publisher  reset_pub= n.advertise<std_msgs::Bool>("reset", 1000);
	//subscribing
	ros::Subscriber sub1 = n.subscribe("imuValues", 1000, imu_updater);
	ros::Subscriber sub2 = n.subscribe("true_position", 1000, gps_updater);
	ros::Subscriber sub5 = n.subscribe("row1", 1000, matrix_1);
	ros::Subscriber sub6 = n.subscribe("row2", 1000, matrix_2);
	ros::Subscriber sub7 = n.subscribe("row3", 1000, matrix_3);
	ros::Subscriber sub8 = n.subscribe("reachPoint", 1000, goal_reached);



	while (ros::ok()){
		geometry_msgs::Vector3 goalposition = geometry_msgs::Vector3();
		ros::Rate loop_rate(10); //frequency setup, here it runs at 10Hz
		resetting(goalposition_pub,reset_pub, loop_rate);
    goalposition.x = goal_x;
    goalposition.y = goal_y;
    goalposition.z = goal_z;
		if (t >= full_time){
			bool finish=true;
			finish_bool.data = finish;
		}
		int i = 1;

		while (goal_x > right_boundary){
			if (i < 60) {
				goal_x = gps_x;
				goal_y = gps_y;
				goal_z = gps_z;
				i++;
			//	ROS_INFO("yeah [%d]",i);
			}
			if (t >= full_time){
				bool finish=true;
				finish_bool.data = finish;
			}
				//ROS_INFO("hello");
				goal_x = start_up(gps_x, backward_constant);
				goalposition.x = goal_x;
				goalposition.y = goal_y;
				goalposition.z = goal_z;
				goalposition_pub.publish(goalposition);
				finish_pub.publish(finish_bool);
				ROS_INFO("time: [%f]",t);
				ROS_INFO("dat [%f],[%f],[%f]",gps_x,gps_y,gps_z);
				ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
				ros::spinOnce(); //neccessary for a later subscription
				loop_rate.sleep(); //sleep until it hits 10Hz publish rate

		}

		while (goal_y < top_boundary){

				//ROS_INFO("fun");
				if (t >= full_time){
					bool finish=true;
					finish_bool.data = finish;
				}
				goal_y = start_up(gps_y, forward_constant);
  			goalposition.x = goal_x;
  			goalposition.y = goal_y;
  			goalposition.z = goal_z;
				goalposition_pub.publish(goalposition);
				finish_pub.publish(finish_bool);
				ROS_INFO("time: [%f]",t);
				ROS_INFO("am [%f],[%f],[%f]",gps_x,gps_y,gps_z);
				ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
				ros::spinOnce(); //neccessary for a later subscription
				loop_rate.sleep(); //sleep until it hits 10Hz publish rate

		}




	//robot is at the top near the water now
	//go to the bow's boundary
	while (goal_x < left_boundary){
		//ROS_INFO("working");
		if (goal_reached){
			goal_x = start_up(gps_x, forward_constant);
			if (t >= full_time){
				bool finish=true;
				finish_bool.data = finish;
			}
			//goal_y = gps_y;
	    goalposition.x = goal_x;
	    goalposition.y = goal_y;
	    goalposition.z = goal_z;
			ROS_INFO("time: [%f]",t);
			ROS_INFO("nice [%f],[%f],[%f]",gps_x,gps_y,gps_z);
			goalposition_pub.publish(goalposition);
			finish_pub.publish(finish_bool);
			}

		ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
		ros::spinOnce(); //neccessary for a later subscription

		loop_rate.sleep(); //sleep until it hits 10Hz publish rate
	}
	finish_pub.publish(finish_bool);


	/* call the algorithm */
  path_plan(goalposition_pub,finish_pub,loop_rate);
	}
	ros::spin();

	return 0;

}
