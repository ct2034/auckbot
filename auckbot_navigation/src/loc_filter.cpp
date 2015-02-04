/*************************************************
loc_filter.cpp                          
------------------
                                              
Copyright 2014 Christian Henkel
Contact: post@henkelchristian.de 
   
This file is part of Auckbot.

Auckbot is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Auckbot is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Auckbot. If not, see http://www.gnu.org/licenses/.             
***************************************************/

// ros
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"

class LocFilter
{


};

LocFilter::LocFilter()
{

}

// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_filter");
  ros::NodeHandle n;

  ROS_INFO("INITIALIZED");
  
  LocFilter lf = LocFilter();
  
  ros::spin();

  return 0;
}