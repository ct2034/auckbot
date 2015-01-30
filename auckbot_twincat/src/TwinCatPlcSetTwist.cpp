  /*
 * This code was built by Christian Scheifele, University of Stuttgart, 2014
 *	cs@christian-scheifele.de
 *
 * It is derived and contains complete segments from:
 * - http://www.binarytides.com/server-client-example-c-sockets-linux/
 *   Server and client example with C sockets on Linux
 * - http://tenouk.com/Module41.html
 *   Network Programming Linux Socket Part 11: TCP Client-Server Code Sample
 * - base_controller.cpp
 * 	 ROS Book
 *
 * Small Sdditions by Christian Henkel - post@henkelchristian.de
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "iostream"
using namespace std;

// TCP/IP Connection
//#include<stdio.h> //printf
#include<string.h>    //strlen
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr

#define MAX_SPEED 9.9

int sock;
struct sockaddr_in server;

// Subscribe Twist Information in ROS
void cmd_velCallback(const geometry_msgs::Twist &twist)
{
	double linVelX = -twist.linear.x;
  if (abs(linVelX) > MAX_SPEED)
  {
    ROS_WARN("Speed in x to big");
    if (linVelX > 0) linVelX = MAX_SPEED;
    else linVelX = -MAX_SPEED;
  }
	double linVelY = -twist.linear.y;
  if (abs(linVelY) > MAX_SPEED)
  {
    ROS_WARN("Speed in y to big");
    if (linVelY > 0) linVelY = MAX_SPEED;
    else linVelY = -MAX_SPEED;
  }
	double linVelZ = 0; // twist.linear.z;

	double angVelX = 0; // twist.angular.x;
	double angVelY = 0; // twist.angular.y;
	double angVelZ = twist.angular.z;

	char message[29];

	// change to mm
	linVelX = linVelX * 1000;
	linVelY = linVelY * 1000;

	// Transformation into wheel velocity
	double R = 100;
	double l1 = 320;
	double l2 = 320;

	double velWheel1 = 1/R * (linVelX + linVelY -(l1+l2)*angVelZ);
	double velWheel2 = 1/R * (linVelX - linVelY +(l1+l2)*angVelZ);
	double velWheel3 = 1/R * (linVelX - linVelY -(l1+l2)*angVelZ);
	double velWheel4 = 1/R * (linVelX + linVelY +(l1+l2)*angVelZ);

	//Create message
	velWheel1 = velWheel1 * 10000;
	velWheel2 = velWheel2 * 10000;
	velWheel3 = velWheel3 * 10000;
	velWheel4 = velWheel4 * 10000;

	char velWheel1Str[8];
	sprintf(velWheel1Str, "%07.0f", velWheel1);
	char velWheel2Str[8];
	sprintf(velWheel2Str, "%07.0f", velWheel2);
	char velWheel3Str[8];
	sprintf(velWheel3Str, "%07.0f", velWheel3);
	char velWheel4Str[8];
	sprintf(velWheel4Str, "%07.0f", velWheel4);


	//velWheel1
	message[0] = velWheel1Str[0];
	message[1] = velWheel1Str[1];
	message[2] = velWheel1Str[2];
	message[3] = velWheel1Str[3];
	message[4] = velWheel1Str[4];
	message[5] = velWheel1Str[5];
	message[6] = velWheel1Str[6];
	//velWheel2
	message[7] = velWheel2Str[0];
	message[8] = velWheel2Str[1];
	message[9] = velWheel2Str[2];
	message[10] = velWheel2Str[3];
	message[11] = velWheel2Str[4];
	message[12] = velWheel2Str[5];
	message[13] = velWheel2Str[6];
	//velWheel3
	message[14] = velWheel3Str[0];
	message[15] = velWheel3Str[1];
	message[16] = velWheel3Str[2];
	message[17] = velWheel3Str[3];
	message[18] = velWheel3Str[4];
	message[19] = velWheel3Str[5];
	message[20] = velWheel3Str[6];
	//velWheel4
	message[21] = velWheel4Str[0];
	message[22] = velWheel4Str[1];
	message[23] = velWheel4Str[2];
	message[24] = velWheel4Str[3];
	message[25] = velWheel4Str[4];
	message[26] = velWheel4Str[5];
	message[27] = velWheel4Str[6];
	//End
	message[28] = '\0';

	//Send Twist to TwinCAT PLC
	if( send(sock , message , strlen(message) , 0) < 0)
	{
		puts("Send failed\n");
	}

	ROS_INFO("> sent to TwinCat: velWheel1 = %d , ..2 = %d , ..3 = %d , ..4 = %d \n", (int) velWheel1, (int) velWheel2, (int) velWheel3, (int) velWheel4);
}

int main(int argc , char **argv)
{
	ros::init(argc, argv, "TwinCatPlcSetTwist");
	ros::NodeHandle n;
	ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);
    
  // Some waiting "intelligence"
  int seconds = 1;
  int waittime = 0;
  int display = 30;

	//Create socket
	sock = socket(AF_INET , SOCK_STREAM , 0);
	if (sock == -1)
	{
		ROS_INFO("Could not create socket\n");
	}
	puts("Socket created\n");

	// Server

	//BECKHOFF
	server.sin_addr.s_addr = inet_addr("192.168.0.3");
	//virtuos
	//server.sin_addr.s_addr = inet_addr("192.168.140.1");
	//

	server.sin_family = AF_INET;
	server.sin_port = htons( 711 );

	//Connect to remote server
	while (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
    ros::spinOnce();
    ros::Duration(seconds).sleep();
    waittime += 1;
    if (waittime%display == 0) {
      ROS_INFO("waited for %d seconds\n", waittime);     
    }
  }
  ROS_INFO("Connected after %d second(s)\n", waittime);     

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	close(sock);
}








