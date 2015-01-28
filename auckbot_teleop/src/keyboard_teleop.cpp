/*************************************************
keyboard_teleop.cpp                          
-------------------
                                              
Copyright 2014 Christian Henkel
Contact: post@henkelchristian.de 
   
This file is part of Auckbot.

Auckbot is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Auckbot is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Auckbot. If not, see http://www.gnu.org/licenses/.             
***************************************************/

#include <ncurses.h>
   
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#define NO_INPUT -1 // TODO: find out actual values
#define SHIFT_Q  81
#define SHIFT_W  87
#define SHIFT_E  69
#define SHIFT_A  65
#define SHIFT_S  83
#define SHIFT_D  68
#define SHIFT    42
#define CTRL_C   3


#define LOOP_RATE 1000
#define ZERO_THRD .1


bool stop();
bool set_speed(float, float, float);
float stop_traj(float grad, float prev);
float acc_traj(float grad, float prev, float goal);

ros::Publisher speedPub;
  
int main(int argc, char** argv)
{
	ROS_INFO("Starting node...");
	ros::init(argc, argv, "keyboard_teleop");
	//ros::Publisher speedPub;
	ros::NodeHandle nh;
	speedPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Rate loop_rate(LOOP_RATE);
	
  float max_sp_lin = .5; 
  float max_sp_rot = .2; 
  float grad = .3; 
  if(!ros::param::get("~max_sp_lin", max_sp_lin)) ROS_ERROR("Can not get param max_sp_lin");
  if(!ros::param::get("~max_sp_rot", max_sp_rot)) ROS_ERROR("Can not get param max_sp_rot");
  if(!ros::param::get("~grad", grad)) ROS_ERROR("Can not get param grad");

// --------------------------

	initscr();
	raw();
	keypad(stdscr, TRUE);
	noecho(); // Switching off echoing using ncurses
			
	char input = NO_INPUT; 
	
	int row, col;
	
	float sp_x = 0;
	float sp_y = 0;
	float sp_th = 0;
	
	char mesg[]="Use SHIFT + W,A,S,D to move, Q,E to turn";
	char mesg2[]="any other key to STOP";
	getmaxyx(stdscr,row,col);
  mvprintw(row/2,   (col-40)/2, "%s", mesg );
  mvprintw(row/2+1, (col-21)/2, "%s", mesg2);

	
	ROS_INFO(".. started and initialized");

		
  while( ros::master::check() & input != CTRL_C )
  {
    input = getch();
    //printw("%d, %c\n", input, input);
    //refresh();
    //cbreak();
          
    switch(input){
      /*case (NO_INPUT):
        sp_x = 0;
        sp_y = 0;
        sp_th = 0;
        stop();
      break;*/
      case (SHIFT_Q):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = acc_traj(grad, sp_th, max_sp_rot);
      break;
      case (SHIFT_W):
        sp_x  = acc_traj(grad, sp_x, max_sp_lin);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = stop_traj(grad, sp_th);
      break;
      case (SHIFT_E):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = acc_traj(grad, sp_th, -1*max_sp_rot);
      break;
      case (SHIFT_A):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = acc_traj(grad, sp_y, max_sp_lin);
        sp_th = stop_traj(grad, sp_th);
      break;
      case (SHIFT_S):
        sp_x  = acc_traj(grad, sp_x, -1*max_sp_lin);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = stop_traj(grad, sp_th);
      break;
      case (SHIFT_D):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = acc_traj(grad, sp_y, -1*max_sp_lin);
        sp_th = stop_traj(grad, sp_th);
      break;
      default:
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = stop_traj(grad, sp_th);
      break;
    }
    
    set_speed(sp_x, sp_y, sp_th);
  
    //loop_rate.sleep();
    ros::spinOnce();
  }
  
  stop();
  endwin();

	ROS_INFO("STOPPING NODE");
  return 0;
}

bool stop(){
  geometry_msgs::Twist msg;
  
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.angular.z = 0.0;
  
  msg.linear.z = 0.0; msg.angular.x = 0.0; msg.angular.y = 0.0;
  
  speedPub.publish(msg);
  
  return true;
}

bool set_speed(float _x, float _y, float _th){
  geometry_msgs::Twist msg;
  
  msg.linear.x = _x;
  msg.linear.y = _y;
  msg.angular.z = _th;
  
  msg.linear.z = 0.0; msg.angular.x = 0.0; msg.angular.y = 0.0;
  
  speedPub.publish(msg);
  
  return true;
}

float stop_traj(float grad, float prev){
  if( prev < ZERO_THRD & prev > -ZERO_THRD ) return 0;
  else return prev * grad;  
}

float acc_traj(float grad, float prev, float goal){
  float diff = goal-prev;
  if( diff < ZERO_THRD & diff > -ZERO_THRD ) return goal;
  else return prev + diff * grad;  
}
  


