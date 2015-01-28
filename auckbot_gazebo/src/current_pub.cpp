/*************************************************
current_pub.cpp                          
---------------
                                              
Copyright 2014 Christian Henkel
Contact: post@henkelchristian.de 
   
This file is part of Auckbot.

Auckbot is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Auckbot is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Auckbot. If not, see http://www.gnu.org/licenses/.             
***************************************************/

#include "ros/ros.h"
#include "auckbot_gazebo/MotorCurrents.h"
#include "geometry_msgs/Twist.h"

//#include <Eigen/Dense>

#define NA 3          // Number of movements
#define NM 4          // Number of motors

#define LENGTH_M 1.2f //Wheel pose length
#define WIDTH_M .4f   //Wheel pose width
#define RADIUS_M 2f   //Wheel radius
#define YOX 1.2f      // y over x coeffiecient (how much more energy does y movement consume?)
#define ACC 2f        // factor for acceleration 

class CurrentPub 
{
  private:
    float speeds[NA];
    float mspeeds[NM];
    float currents[NM];
    float indize[NM][NA];
    ros::Publisher publisher;
    ros::Time timeLast;
 
  public:
    CurrentPub();
    void velocityCallback(const geometry_msgs::Twist& msg);
    void setPub(ros::Publisher publisher);
    //void setIndize(float* indize);
    
  private:
    void setCurrents(float* speeds, float duration);
};

// class functions
CurrentPub::CurrentPub()
{
  // Influence on motor 1
  indize[0][0] =  1.0;             // <--  from dx
  indize[0][1] =  1.0*YOX;         // <--  from dy  (y speed)
  indize[0][2] = -(LENGTH_M + WIDTH_M); // from dth (rotational speed)
  
  // motor 2
  indize[1][0] =  1.0;
  indize[1][1] = -1.0*YOX;
  indize[1][2] =  (LENGTH_M + WIDTH_M);
  
  indize[2][0] =  1.0;
  indize[2][1] = -1.0*YOX;
  indize[2][2] = -(LENGTH_M + WIDTH_M);
  
  indize[3][0] =  1.0;
  indize[3][1] =  1.0*YOX;
  indize[3][2] =  (LENGTH_M + WIDTH_M);
}
  
void CurrentPub::velocityCallback(const geometry_msgs::Twist& speedMsg)
{
  float _secDuration;
  ros::Time _now;
  
  //ROS_INFO("I heard x-speed = %f, Duration: %f s", speedMsg.linear.x, secDuration);
  
  _now = ros::Time::now();
  _secDuration = _now.toSec() - timeLast.toSec();
  speeds[0] = speedMsg.linear.x;
  speeds[1] = speedMsg.linear.y;
  speeds[2] = speedMsg.angular.z;
  
  setCurrents(speeds, _secDuration);
  
  auckbot_gazebo::MotorCurrents currentMsg;
  currentMsg.time = _now;
  currentMsg.motor_1 = currents[0];
  currentMsg.motor_2 = currents[1];
  currentMsg.motor_3 = currents[2];
  currentMsg.motor_4 = currents[3];
  
  publisher.publish(currentMsg);
  timeLast = _now;
}

void CurrentPub::setPub(ros::Publisher publisher_)
{
  publisher = publisher_;
}

void CurrentPub::setCurrents(float* speeds, float duration)
{
  int im, ia;
  float _mspeeds[NM];
  float _maccele[NM];
  float _scurrents[NM];
  for(im=0; im<NM; im++)
  {
    _mspeeds[im] = 0;
    for(ia=0; ia<NA; ia++)
    {
      _mspeeds[im] += (indize[im][ia] * speeds[ia]);
    }
    
    _maccele[im] = (_mspeeds[im]-mspeeds[im])/duration;
    mspeeds[im] = _mspeeds[im]; // store old values for next time
    
    _scurrents[im] = sqrt(_mspeeds[im]*_mspeeds[im]) +_maccele[im];
    currents[im] = sqrt(_scurrents[im]*_scurrents[im]);
  }
  
}

// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "current_pub");
  ros::NodeHandle n;
  
  CurrentPub cb = CurrentPub();
  cb.setPub( n.advertise<auckbot_gazebo::MotorCurrents>("current", 1000) );
  //cb.setIndize( {8} );
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &CurrentPub::velocityCallback, &cb);
  
  ros::spin();

  return 0;
}

