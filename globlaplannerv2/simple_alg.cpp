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
double gps_w;

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

int forward_constant = 1;
int backward_constant = -1;

extern bool boundary_stern = false;
extern bool boundary_keel = false;
extern bool boundary_bow = false;
extern bool boundary_water_surf = false;

bool boundary;
bool reached_goal;
bool goingBowToStern;

geometry_msgs::Vector3 goalposition;

// below is setup for the functions that listen if any messages were published


//get IMU data
void imu_updater(const geometry_msgs::Vector3::ConstPtr& robot_orient){

	imu_x = floor(robot_orient->x); //pitch
	imu_y = floor(robot_orient->y); //yaw
	imu_z = floor(robot_orient->z); //roll

	//ROS_INFO("Received New IMU readings @ [%f],[%f],[%f]", imu_x, imu_y, imu_z);
}


//get GPS pose from the beacons
void gps_updater(const geometry_msgs::Quaternion::ConstPtr& robot_pose){

	gps_x = floor(robot_pose->x);
	gps_y = floor(robot_pose->y);
	gps_z = floor(robot_pose->z);
	gps_w = floor(robot_pose->w);

	//ROS_INFO("Received New GPS Position @ [%f],[%f],[%f]", gps_x, gps_y, gps_z);
}

void goal_reached(const std_msgs::Bool::ConstPtr& goal_reached_bool){
	reached_goal = goal_reached_bool->data;
}

//get boundaries from the beacons
/*void boundary_updater(const geometry_msgs::Quaternion::ConstPtr& boundary_bool){

	float boundary_x = boundary_bool->x;
	float boundary_y = boundary_bool->y;
	float boundary_z = boundary_bool->z;
	float boundary_w = boundary_bool->w;
	//double gps_x, gps_y, gps_z;

	//ROS_INFO("her is x: [%f]",boundary_x);
	//ROS_INFO("her is x: [%f]",boundary_y);
	//boundary_x
	//boundary_x




	/*if (boundary_stern == true){
		ROS_INFO("Boundary on the stern is reached.");
	}

	if (boundary_stern == false){
		ROS_INFO("Boundary on the stern not reached yet.");
	}

}*/


/*void normvector_updater(const geometry_msgs::Vector3::ConstPtr& norm_vector){

	norm_x = floor(norm_vector->x);
	norm_y = floor(norm_vector->y);
	norm_z = floor(norm_vector->z);

	//ROS_INFO("Received New Norm Vector readings @ [%f],[%f],[%f]", norm_x, norm_y, norm_z);
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
double start_up(double robot_pose_gps_reading, double go_forward_constant){

	double goal_height = robot_pose_gps_reading + go_forward_constant;

	return goal_height;
}

double goDownTheBoundary(bool boundary,ros::Publisher goalposition_pub, ros::Rate loop_rate){

	ROS_INFO("boundary");

	if (goal_x >= 54 || goal_x <= -47){
		ROS_INFO("going down");
		goal_y = goal_y + backward_constant;
	}

	ROS_INFO("boundary [%f],[%f],[%f]",gps_x,gps_y,gps_z);
	ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
	goalposition.x = gps_x;
	goalposition.y = goal_y;
	goalposition.z = gps_z;

	goalposition_pub.publish(goalposition);


	goingBowToStern = !goingBowToStern;
	boundary = false;

	ros::spinOnce(); //neccessary for a later subscription
	loop_rate.sleep(); //sleep until it hits 10Hz publish rate

	return goal_y;
}


//clean the boat
void crossBoat(bool goingBowToStern,ros::Publisher goalposition_pub, ros::Rate loop_rate){

	ROS_INFO("cleaning");
	while (gps_x <= 54  || gps_x >= -47){

		if (goal_x >= 54 || goal_x <= -47){
			goDownTheBoundary(true, goalposition_pub, loop_rate);
		};

		if (goingBowToStern){
			ROS_INFO("going right");
			goal_x = gps_x + backward_constant;
			ROS_INFO("right [%f],[%f],[%f]",gps_x,gps_y,gps_z);
			ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
			if (goal_x == -47){
				goingBowToStern = false;
			}
		}
		else {
			ROS_INFO("going left");
			goal_x = gps_x + forward_constant;
			ROS_INFO("left [%f],[%f],[%f]",gps_x,gps_y,gps_z);
			ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
		}
		//go down the boundary
		goalposition.x = goal_x;
    goalposition.y = goal_y;
    goalposition.z = gps_z;

		goalposition_pub.publish(goalposition);
		ros::spinOnce(); //neccessary for a later subscription

		loop_rate.sleep(); //sleep until it hits 10Hz publish rate
	}
}

void path_plan(ros::Publisher goalposition_pub, ros::Rate loop_rate){

	//bool visited = false;
	bool goingBowToStern = true;
	bool doneWithSide = false;
	bool boundary = true;
	//go straight to the goal
	while (!doneWithSide){
		ROS_INFO("wonky donky");
		crossBoat(true, goalposition_pub, loop_rate);
    /*goalposition.x = goal_x;
    goalposition.y = goal_y;
    goalposition.z = gps_z;
    goalposition_pub.publish(goalposition); //here we publish our message*/

    while (reached_goal==false || boundary_keel && goal_y <= -15){
			ROS_INFO("im here now");
			/*ros::spinOnce(); //neccessary for a later subscription
			loop_rate.sleep(); //sleep until it hits 10Hz publish rate*/
    }
    goingBowToStern = !goingBowToStern;
		/*if (visited){
		ROS_INFO("or here?");
		crossBoat(goingBowToStern,goalposition_pub,loop_rate);
		//doneWithSide = true;// go to the other side of boat			+
		}*/
		}

		ros::spinOnce(); //neccessary for a later subscription

		loop_rate.sleep(); //sleep until it hits 10Hz publish rate

		//go straight at the keel
		/*if (boundary_keel == true || goal_y <=-15 && boundary_bow == true || goal_x >= 54 || boundary_stern == true || goal_x <= -47 && boundary_keel == true || goal_y <= -16){
			visited = true;
		}*/

}

// generate two point on the robot to get plan
// get pitch, roll, yaw from imu, get the position of the robot
// generate a point in the middle of the robot and two points in front
// put those values in a rotation matrix, so you see the orientation and translation
/*geometry_msgs::Vector3 generate_plane (double robot_orient_imu_pitch, double robot_orient_imu_roll, double robot_orient_imu_yaw, double robot_middle_x, double robot_middle_y, double robot_middle_z){

}*/

/* <summary> function returning 'vector cross_p' product, taking 'vector v1' and 'vector v2' vectors as parameters </summary>
   <param name="v1"> vector between middle of the robot and point above the right front wheel </param>
   <param name="v2"> vector between middle of the robot and point above the left front wheel </param>
   <param name="cross_p"> return normal vector between v1 and v2 </param> */
/*void crossproduct(vector v1, vector v2, vector cross_p)

{

 cross_p.i = (v1.j*v2.k) - (v1.k*v2.j);
 cross_p.j = -((v1.i*v2.k) - (v1.k*v2.i));
 cross_p.k = (v1.i*v2.j) - (v1.j*v2.i);

 //cout <<"crossproduct = "<<cross_p.i<<"i"<<cross_p.j<<"j"<<cross_p.k<<"k"<<endl;

}*/

// generate a norm vector
// generate the three previous points, take a cross-product = norm vector (we want to return this norm vector)

// retrieve norm vector from the bounadry
// subscribe to the norm vector from the boundary DONE

// evaluate the norm vectors aginst each others
// compare two normal vectors if they are equal to each other, if true then you are on the keel

// zy on the side (x, y) keel the axis zx is (x, y) - how the world is set up

// note 1: 1 is true zero is false in the boundary

// note 2: special case dist to gps boundarys


int main(int argc, char **argv){
ROS_INFO("here");
	ros::init(argc, argv, "MY_GLOBAL_PLANNER"); //initialise ROS topic for publishing and subscribing
	ros::NodeHandle n; //initialise communication

	//publishing to the local planner the goal position
  ros::Publisher goalposition_pub = n.advertise<geometry_msgs::Vector3>("goal", 1000);

	//subscribing
	ros::Subscriber sub1 = n.subscribe("imuValues", 1000, imu_updater);
	ros::Subscriber sub2 = n.subscribe("true_position", 1000, gps_updater);
	//ros::Subscriber sub3 = n.subscribe("boundary", 1000, boundary_updater);
	//ros::Subscriber sub4 = n.subscribe("norm_vector", 1000, normvector_updater);
	ros::Subscriber sub5 = n.subscribe("row1", 1000, matrix_1);
	ros::Subscriber sub6 = n.subscribe("row2", 1000, matrix_2);
	ros::Subscriber sub7 = n.subscribe("row3", 1000, matrix_3);

	ros::Subscriber sub8 = n.subscribe("reachPoint", 1000, goal_reached);



	while (ros::ok()){

		boundary_water_surf = false;
		boundary_bow = false;
		boundary_keel = false;
		boundary_stern = false;

		if (gps_x <= -48.7 ){
		 		boundary_stern = true;
			 	}

		if (gps_y <= -16){
		 		boundary_keel = true;
			 	}

		if (gps_x >= 54){
		 		boundary_bow = true;
			 	}

		if (gps_y >= 27){
		 		boundary_water_surf = true;
			 		ROS_INFO("Got it");
			 	}
		//ROS_INFO("i [%f],[%f],[%f]",goal_x,goal_y,goal_z);
		geometry_msgs::Vector3 goalposition = geometry_msgs::Vector3();
		ros::Rate loop_rate(10); //frequency setup, here it runs at 10Hz
    goalposition.x = goal_x;
    goalposition.y = goal_y;
    goalposition.z = goal_z;
		int i = 1;
		//boundary_water_surf = true;
    //go to the water's surface

		while (goal_x >= -47){
			if (i < 50) {
				goal_x = gps_x;
				goal_y = gps_y;
				goal_z = gps_z;
				i++;
				ROS_INFO("yeah [%d]",i);
			}
				ROS_INFO("hello");
				goal_x = start_up(gps_x, backward_constant);
				goalposition.x = goal_x;
				goalposition.y = goal_y;
				goalposition.z = goal_z;
				goalposition_pub.publish(goalposition);
				ROS_INFO("dat [%f],[%f],[%f]",gps_x,gps_y,gps_z);
				ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
				ros::spinOnce(); //neccessary for a later subscription
				loop_rate.sleep(); //sleep until it hits 10Hz publish rate

		}

		while (boundary_water_surf == false && goal_y < 27){

				ROS_INFO("fun");
				goal_y = start_up(gps_y, forward_constant);
  			goalposition.x = goal_x;
  			goalposition.y = goal_y;
  			goalposition.z = goal_z;
				goalposition_pub.publish(goalposition);
				ROS_INFO("am [%f],[%f],[%f]",gps_x,gps_y,gps_z);
				ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
				ros::spinOnce(); //neccessary for a later subscription
				loop_rate.sleep(); //sleep until it hits 10Hz publish rate

		}




	//robot is at the top near the water now
	//go to the bow's boundary
	while (boundary_bow == false && goal_x < 54){
		ROS_INFO("working");
		if (goal_reached){
			goal_x = start_up(gps_x, forward_constant);
			//goal_y = gps_y;
	    goalposition.x = goal_x;
	    goalposition.y = goal_y;
	    goalposition.z = goal_z;
			ROS_INFO("nice [%f],[%f],[%f]",gps_x,gps_y,gps_z);
			goalposition_pub.publish(goalposition);
			}

		ROS_INFO("goal [%f],[%f],[%f]",goal_x,goal_y,goal_z);
		ros::spinOnce(); //neccessary for a later subscription

		loop_rate.sleep(); //sleep until it hits 10Hz publish rate
	}



	/* call the algorithm */
  path_plan(goalposition_pub,loop_rate);
	}
	ros::spin();

	return 0;

}
