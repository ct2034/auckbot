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
 * Small Sdditions by Christian Henkel - post@henkelchristian.de
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


int main(int argc , char **argv)
{
	ros::init(argc, argv, "TwinCatPlcGetOdom");

	ros::NodeHandle n;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;


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


	// robot start position -> regarding to "odom" coordinate frame
//	double x = 0.0;
//	double y = 0.0;
//	double th = 0.0;
//
//	double vx = 0.1;
//	double vy = -0.1;
//	double vth = 0.1;


	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// We'll publish odometry information at a rate of 1Hz in this example
	// to make introspection easy
	ros::Rate r(1.0);

	while(n.ok()){

		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		//receive odometry message from TwinCAT
	     int totalcnt = 0;
	     int BufferLength = 49;
	     int rc;

	     while(totalcnt < BufferLength)
	     {
	     	rc = read(sock, &twinCatMessage[totalcnt], BufferLength-totalcnt);
	     	if(rc < 0)
	     	{
	     		puts("FAULT\n");
	     		perror("Client-read() error");
	     		close(sock);
	     		exit(-1);
	     	}
	     	else if (rc == 0)
	     	{
	     		puts("FAULT\n");
	     		printf("Server program has issued a close()\n");
	     		close(sock);
	     		exit(-1);
	     	}
	     	else
	     		totalcnt += rc;
	     }

	     char robotPosXStr[7] = {twinCatMessage[0], twinCatMessage[1], twinCatMessage[2], twinCatMessage[3], twinCatMessage[4], twinCatMessage[5], '\0'};
	     char robotPosYStr[7] = {twinCatMessage[7], twinCatMessage[8], twinCatMessage[9], twinCatMessage[10], twinCatMessage[11], twinCatMessage[12], '\0'};
	     char robotPosThetaStr[7] = {twinCatMessage[14], twinCatMessage[15], twinCatMessage[16], twinCatMessage[17], twinCatMessage[18], twinCatMessage[19], '\0'};
	     char robotVelXStr[7] = {twinCatMessage[21], twinCatMessage[22], twinCatMessage[23], twinCatMessage[24], twinCatMessage[25], twinCatMessage[26], '\0'};
	     char robotVelYStr[7] = {twinCatMessage[28], twinCatMessage[29], twinCatMessage[30], twinCatMessage[31], twinCatMessage[32], twinCatMessage[33], '\0'};
	     char robotVelThetaStr[7] = {twinCatMessage[35], twinCatMessage[36], twinCatMessage[37], twinCatMessage[38], twinCatMessage[39], twinCatMessage[40], '\0'};

	     char robotPosX_signStr[2] = {twinCatMessage[42], '\0'};
	     char robotPosY_signStr[2] = {twinCatMessage[43], '\0'};
	     char robotPosTheta_signStr[2] = {twinCatMessage[44], '\0'};
	     char robotVelX_signStr[2] = {twinCatMessage[45], '\0'};
	     char robotVelY_signStr[2] = {twinCatMessage[46], '\0'};
	     char robotVelTheta_signStr[2] = {twinCatMessage[47], '\0'};

	     double robotPosX = (strtod(robotPosXStr, NULL))/1000;
	     double robotPosY = (strtod(robotPosYStr, NULL))/1000;
	     double robotPosTheta = strtod(robotPosThetaStr, NULL)/10;
	     double robotVelX = (strtod(robotVelXStr, NULL))/1000;
	     double robotVelY = (strtod(robotVelYStr, NULL))/1000;
	     double robotVelTheta = strtod(robotVelThetaStr, NULL)/10;

	     double robotPosX_sign = strtod(robotPosX_signStr, NULL);
	     double robotPosY_sign = strtod(robotPosY_signStr, NULL);
	     double robotPosTheta_sign = strtod(robotPosTheta_signStr, NULL);
	     double robotVelX_sign = strtod(robotVelX_signStr, NULL);
	     double robotVelY_sign = strtod(robotVelY_signStr, NULL);
	     double robotVelTheta_sign = strtod(robotVelTheta_signStr, NULL);

	     if(robotPosX_sign == 1)
	     	     	{
	    	 	 	 	 robotPosX = robotPosX*(-1);
	     	     	}
	     if(robotPosY_sign == 1)
	     	     	{
	    	 robotPosY = robotPosY*(-1);
	     	     	}
	     if(robotPosTheta_sign == 1)
	     	     	{
	    	 robotPosTheta = robotPosTheta*(-1);
	     	     	}
	     if(robotVelX_sign == 1)
	     	     	{
	    	 robotVelX = robotVelX*(-1);
	     	     	}
	     if(robotVelY_sign == 1)
	     	     	{
	    	 robotVelY = robotVelY*(-1);
	     	     	}
	     if(robotVelTheta_sign == 1)
	     	     	{
	    	 robotVelTheta = robotVelTheta*(-1);
	     	     	}


		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPosTheta);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = robotPosX;
		odom_trans.transform.translation.y = robotPosY;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = robotPosX;
		odom.pose.pose.position.y = robotPosY;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = robotVelX;
		odom.twist.twist.linear.y = robotVelY;
		odom.twist.twist.angular.z = robotVelTheta;

		//publish the message
		odom_pub.publish(odom);
		printf("...Odometry published. RobotPosX = %f , RobotPosY = %f , RobotPosTheta = %f, RobotVelX = %f , RobotVelY = %f , RobotVelTheta = %f\n\n", robotPosX, robotPosY, robotPosTheta, robotVelX, robotVelY, robotVelTheta);

		last_time = current_time;
		//r.sleep();
	}

}
