/*************************************************
automatron.cpp                          
-------------------
                                              
Copyright 2014 Christian Henkel
Contact: post@henkelchristian.de 
   
This file is part of Auckbot.

Auckbot is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Auckbot is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Auckbot. If not, see http://www.gnu.org/licenses/.             
***************************************************/

#include <ros/ros.h> 
#include <ros/time.h>  

#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>

#define NO_TRIPS	20
#define NO_GOALS	2
//#define NO_CONFIGS	4 

#define WAIT_T		5 //s

float x[NO_GOALS];
float y[NO_GOALS];
float z[NO_GOALS];
float w[NO_GOALS];
int i;

ros::Subscriber resultSub;
ros::Publisher pub;

void resultCallback(const move_base_msgs::MoveBaseActionResult msg);
void sendNextGoal(void);

void sendNextGoal(void) {
	ROS_INFO("new goal (%d)", i);

	if(i>NO_TRIPS) ros::shutdown();

	for (int iw = WAIT_T; iw>=0; iw--) {
		ROS_INFO("waiting ... (%d)", iw);
		ros::Duration(1).sleep();
	}

	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "/map";
	goal.pose.position.x = x[i%NO_GOALS];
    goal.pose.position.y = y[i%NO_GOALS];
    goal.pose.orientation.z = z[i%NO_GOALS];
    goal.pose.orientation.w = w[i%NO_GOALS];
	pub.publish(goal);

	i++;
}

void resultCallback(const move_base_msgs::MoveBaseActionResult msg) {
	sendNextGoal();
	// ROS_INFO("status recieved");
}

int main(int argc, char** argv) { 
	x[0] = 0;
	x[1] = 7;
	y[0] = 0;
	y[1] = 7;
	z[0] = 0;
	z[1] = 1;
	w[0] = 1;
	w[1] = 0;
	i = 0;

	ROS_INFO("Starting node...");
	ros::init(argc, argv, "automatron");
	ros::NodeHandle nh;
	  
	resultSub = nh.subscribe("/move_base/result", \
		1000, resultCallback);
	pub = nh.advertise<geometry_msgs::PoseStamped> \
		("/move_base_simple/goal", 5);
	  
	sendNextGoal();

	while (nh.ok()){
		ros::spinOnce();
	}

	ROS_INFO("Stopping Node");
	return 0;
}