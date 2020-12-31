/*Include*/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <ros/console.h>
#include <sstream>
#include <iostream>
#include <random>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
/*!Include!*/

/*Declare*/
void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg);
extern geometry_msgs::Vector3 objective = geometry_msgs::Vector3();
void globalCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
extern geometry_msgs::Quaternion global_position = geometry_msgs::Quaternion();
void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg);
extern geometry_msgs::Vector3 imu = geometry_msgs::Vector3();
void row1Callback(const geometry_msgs::Vector3::ConstPtr& msg);
void row2Callback(const geometry_msgs::Vector3::ConstPtr& msg);
void row3Callback(const geometry_msgs::Vector3::ConstPtr& msg);
extern double rotationMatrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
extern double rM[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

// geometry_msgs::Vector3Stamped addNoise(geometry_msgs::Vector3Stamped point);
std_msgs::Bool reachedQ(geometry_msgs::Vector3 objective, geometry_msgs::Quaternion position);
void head(double initial[], geometry_msgs::Quaternion position, geometry_msgs::Vector3 &heading);
void multiply(double trans[3][3], double point[3], double result[3]);
void subtract(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2, double result[3]);
/*!Declare!*/


int main(int argc, char **argv) {
  ros::init(argc, argv, "planner"); // topic name remappings
  ros::NodeHandle n; // Node

  /*Publishers*/
  ros::Publisher heading_pub = n.advertise<geometry_msgs::Vector3>("heading", 1000);
  ros::Publisher reached_pub = n.advertise<std_msgs::Bool>("reachPoint", 1);
  /*!Publishers!*/

  /*Subscribers*/
  ros::Subscriber objective_sub = n.subscribe("goal", 1000, objectiveCallback);
  ros::Subscriber global_position_sub = n.subscribe("path", 1000, globalCallback);
  ros::Subscriber imu_sub = n.subscribe("imuValues", 1000, imuCallback);
  ros::Subscriber row1_sub = n.subscribe("row1",2,row1Callback);
  ros::Subscriber row2_sub = n.subscribe("row2",2,row2Callback);
  ros::Subscriber row3_sub = n.subscribe("row3",2,row3Callback);
  /*!Subscribers!*/

  ros::Rate loop_rate(50); // time delay in Hz (50Hz)

  /*Declare*/
  std_msgs::Bool reached;
  geometry_msgs::Vector3 heading = geometry_msgs::Vector3();
  double initial_rotation[3] = {0,0,0};
  /*!Declare!*/

  while (ros::ok()) {

    reached = reachedQ(objective, global_position);
    reached_pub.publish(reached);

    head(initial_rotation, global_position, heading);
    heading_pub.publish(heading);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

/*Functions*/

/*Get ROS information into main*/
void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg){
  objective = *msg;
}

void globalCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
  global_position = *msg;
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  imu = *msg;
}

void row1Callback(const geometry_msgs::Vector3::ConstPtr& msg) {
	geometry_msgs::Vector3 v = *msg;
	rM[0][0] = v.x;
	rM[0][1] = v.y;
	rM[0][2] = v.z;
}

void row2Callback(const geometry_msgs::Vector3::ConstPtr& msg) {
	geometry_msgs::Vector3 v = *msg;
	rM[1][0] = v.x;
	rM[1][1] = v.y;
	rM[1][2] = v.z;
}

void row3Callback(const geometry_msgs::Vector3::ConstPtr& msg) {
	geometry_msgs::Vector3 v = *msg;
	rM[2][0] = v.x;
	rM[2][1] = v.y;
	rM[2][2] = v.z;

	//rotationMatrix[][] = rM[][];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rotationMatrix[i][j] = rM[i][j];
		}
	}
}
/*!Get ROS information into main!*/

std_msgs::Bool reachedQ(geometry_msgs::Vector3 objective, geometry_msgs::Quaternion position) { // Check if the robot has arrived to the current goal
  double error = .25; // radious to accept reached
	double rotated[3];
  ROS_DEBUG("x: %d y: %d z: %d ", (objective.x,objective.y,objective.z));
  /*Difference on each coordinate*/
  double x = objective.x - position.x;
  double y = objective.y - position.y;
  double z = objective.z - position.z;
	double global_dist[3] = {x, y, z};

	multiply(rotationMatrix, global_dist, rotated);
  /*!Difference on each coordinate!*/

  std_msgs::Bool reached = std_msgs::Bool();
  reached.data = error > std::sqrt(pow(rotated[0],2) + pow(rotated[2],2));
  return reached;
}

void head(double initial[], geometry_msgs::Quaternion position, geometry_msgs::Vector3 &heading) { // calculate the turning angle and distance to the goal
  /*Declare*/
  double diff[3];
  double diff_rot[3];
  geometry_msgs::Vector3 pos;
  double angle;
  /*!Declare!*/

  geometry_msgs::Vector3 pub_imu;

  pos = geometry_msgs::Vector3();
  pos.x = position.x;
  pos.y = position.y;
  pos.z = position.z;

  /*Get vector rotated*/
  subtract(objective, pos, diff);
  multiply(rotationMatrix, diff, diff_rot);
  /*Get vector rotated*/

  angle = atan2(-diff_rot[2],diff_rot[0]); // angle
  heading.y = angle;
  heading.x = std::sqrt(pow(diff_rot[0],2) + pow(diff_rot[2],2));
}

void multiply(double trans[3][3], double point[3], double result[3]) { // Multiply a [3x3] matrix by [3x1]
  for (int i=0; i<3; i++) {
    double sum = 0;
    for (int j=0; j<3; j++) {
      sum += trans[i][j]*point[j];
    }
    result[i] = sum;
  }
}

void subtract(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2, double result[3]) { // subtract two [3x1] matrix
  result[0] = vec1.x - vec2.x;
  result[1] = vec1.y - vec2.y;
  result[2] = vec1.z - vec2.z;
}
/*!Functions!*/
