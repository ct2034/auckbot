/*
 * This code was built by Christian Scheifele, University of Stuttgart, 2014
 *	cs@christian-scheifele.de
 *
 * It is derived and contains complete segments from:
 * - http://www.binarytides.com/server-client-example-c-sockets-linux/
 *   Server and client example with C sockets on Linux
 * - http://tenouk.com/Module41.html
 *   Network Programming Linux Socket Part 11: TCP Client-Server Code Sample
 * - http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 *   Publishing Odometry Information over ROS
 *
 * Small additions by Christian Henkel - post@henkelchristian.de
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

// TCP/IP Connection
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr

// Publishing Odometry Information over ROS
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// Split a string with boost
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#define MMPM 1000
#define MAX_ACC 100
#define PI 3.14159265359

int main(int argc , char **argv)
{
	ros::init(argc, argv, "TwinCatPlcGetOdom");

	ros::NodeHandle n;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

  // do we want tf?
  bool publish_tf = false;
  if(!ros::param::get("~publish_tf", publish_tf)) ROS_ERROR("Can not get param publish_tf, using default (false)");
  ROS_INFO("publish_tf: %d", publish_tf);

	// TCP/IP Connection - variables
    int sock, read_size;
    struct sockaddr_in server;
    char twinCatMessage[48];
    
    // Some waiting "intelligence"
    int seconds = 1;
    int waittime = 0;
    int display = 30;

    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1)
    {
        printf("Could not create socket\n");
    }
    puts("Socket created\n");

    // Server

    //BECKHOFF
	server.sin_addr.s_addr = inet_addr("192.168.0.3");
	//virtuos
	//server.sin_addr.s_addr = inet_addr("192.168.140.1");
	//
    server.sin_family = AF_INET;
    server.sin_port = htons( 712 );

    //Connect to remote server
    while (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        ros::spinOnce();
        ros::Duration(seconds).sleep();
        waittime += 1;
        if (waittime%display == 0) {
            printf("waited for %d seconds\n", waittime);     
        }
    }
    printf("Connected after %d second(s)\n", waittime);     

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// We'll publish odometry information at a rate of 1Hz in this example
	// to make introspection easy
	ros::Rate r(1.0);

	while(n.ok()){
		//init
		ros::spinOnce();               // check for incoming messages

		double robotPosX = 0;
		double robotPosY = 0;
		double robotPosTheta = 0;
		double robotVelX = 0;
		double robotVelY = 0;
		double robotVelTheta = 0;

		//receive odometry message from TwinCAT
		int totalcnt = 0;
		int BufferLength = 46;
		int rc;

    memset(twinCatMessage, 0, BufferLength);

		rc = read(sock, twinCatMessage, BufferLength);
    rc = strlen(twinCatMessage);

    current_time = ros::Time::now();  

	  if(rc < BufferLength) {
      memset(twinCatMessage, 0, BufferLength);
    } else {
      current_time = ros::Time::now();      

      //ROS_INFO("alright: %s", twinCatMessage);

      std::string str = std::string(twinCatMessage);
      std::vector<std::string> tokens;

      boost::algorithm::split(tokens, str, boost::algorithm::is_any_of(" ")); // here it is

      //ROS_INFO("no2: >%s<, no3: >%s<", tokens[2].c_str(), tokens[3].c_str());
      
		  char robotPosXStr[7];
      tokens[0].copy(robotPosXStr, 7, 0);      
		  char robotPosYStr[7];
      tokens[1].copy(robotPosYStr, 7, 0);
		  char robotPosThetaStr[7];
      tokens[2].copy(robotPosThetaStr, 7, 0);
		  char robotVelXStr[7];
      tokens[3].copy(robotVelXStr, 7, 0);
		  char robotVelYStr[7];
      tokens[4].copy(robotVelYStr, 7, 0);
		  char robotVelThetaStr[7];
      tokens[5].copy(robotVelThetaStr, 7, 0);
    
		  char robotPosX_signStr[2] = {robotPosXStr[0], '\0'};
		  char robotPosY_signStr[2] = {robotPosYStr[0], '\0'};
		  char robotPosTheta_signStr[2] = {robotPosThetaStr[0], '\0'};
		  char robotVelX_signStr[2] = {robotVelXStr[0], '\0'};
		  char robotVelY_signStr[2] = {robotVelYStr[0], '\0'};
		  char robotVelTheta_signStr[2] = {robotVelThetaStr[0], '\0'};

      ROS_INFO("robotPosX_signStr: >%s<, robotVelY_signStr: >%s<", robotPosX_signStr, robotVelY_signStr);

		  double oldRobotPosX = robotPosX;
		  double oldRobotPosY = robotPosY;
		  double oldRobotPosTheta = robotPosTheta;
		  double oldRobotVelX = robotVelX;
		  double oldRobotVelY = robotVelY;
		  double oldRobotVelTheta = robotVelTheta;

		  robotPosX = (strtod(robotPosXStr, NULL))/MMPM;
		  robotPosY = (strtod(robotPosYStr, NULL))/MMPM;
		  robotPosTheta = strtod(robotPosThetaStr, NULL)/18000*PI;
		  robotVelX = (strtod(robotVelXStr, NULL))/MMPM;
		  robotVelY = (strtod(robotVelYStr, NULL))/MMPM;
		  robotVelTheta = strtod(robotVelThetaStr, NULL)/18000*PI;

      //ROS_INFO("%s, %f, %f", robotPosThetaStr, robotPosTheta, robotPosTheta/180*2*PI);

		  double robotPosX_sign = strtod(robotPosX_signStr, NULL);
		  double robotPosY_sign = strtod(robotPosY_signStr, NULL);
		  double robotPosTheta_sign = strtod(robotPosTheta_signStr, NULL);
		  double robotVelX_sign = strtod(robotVelX_signStr, NULL);
		  double robotVelY_sign = strtod(robotVelY_signStr, NULL);
		  double robotVelTheta_sign = strtod(robotVelTheta_signStr, NULL);

		  if(robotPosX_sign == 1) robotPosX = robotPosX*(-1);
		  if(robotPosY_sign != 1) robotPosY = robotPosY*(-1);
		  if(robotPosTheta_sign == 1) robotPosTheta = robotPosTheta*(-1);
		  if(robotVelX_sign == 1) robotVelX = robotVelX*(-1);
		  if(robotVelY_sign != 1) robotVelY = robotVelY*(-1);
		  if(robotVelTheta_sign == 1) robotVelTheta = robotVelTheta*(-1);
			   	
		  //since all odometry is 6DOF we'll need a quaternion created from yaw
		  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPosTheta);

		  //first, we'll publish the transform over tf
		  geometry_msgs::TransformStamped odom_trans;
		  odom_trans.header.stamp = current_time;
		  odom_trans.header.frame_id = "/odom";
		  odom_trans.child_frame_id = "/base_link";

		  odom_trans.transform.translation.x = robotPosX;
		  odom_trans.transform.translation.y = robotPosY;
		  odom_trans.transform.translation.z = 0.0;
		  odom_trans.transform.rotation = odom_quat;

      
      if(publish_tf) {    
		    //send the transform
		    odom_broadcaster.sendTransform(odom_trans);
      }

		  //next, we'll publish the odometry message over ROS
		  nav_msgs::Odometry odom;
		  odom.header.stamp = current_time;
		  odom.header.frame_id = "/odom";
		  odom.child_frame_id = "/base_link";

		  //set the position
		  odom.pose.pose.position.x = robotPosX;
		  odom.pose.pose.position.y = robotPosY;
		  odom.pose.pose.position.z = 0.0;
		  odom.pose.pose.orientation = odom_quat;
      odom.pose.covariance[0] = 1;
      odom.pose.covariance[7] = 1;
      odom.pose.covariance[14] = 1;
      odom.pose.covariance[21] = 1;
      odom.pose.covariance[28] = 1;
      odom.pose.covariance[35] = 1;

		  //set the velocity from TwinCat
		  odom.twist.twist.linear.x = robotVelX;
		  odom.twist.twist.linear.y = robotVelY;
		  odom.twist.twist.angular.z = robotVelTheta;
      odom.twist.covariance[0] = 1;
      odom.twist.covariance[7] = 1;
      odom.twist.covariance[14] = 1;
      odom.twist.covariance[21] = 1;
      odom.twist.covariance[28] = 1;
      odom.twist.covariance[35] = 1;
		  

		  //publish the message
		  odom_pub.publish(odom);
		  ROS_INFO("...Odometry published. %f %f %f %f %f %f\n", robotPosX, robotPosY, robotPosTheta, robotVelX, robotVelY, robotVelTheta);

		  last_time = current_time;
		  r.sleep();

    }
	}

}
