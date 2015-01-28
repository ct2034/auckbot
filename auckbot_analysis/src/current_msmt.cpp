/*************************************************
current_msmt.cpp                          
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
#include "cob_phidgets/AnalogSensor.h"

#define PHIDGET_DC_DIV 13.2
#define PHIDGET_DC_SUB 37.8787
#define PHIDGET_AC_MUL 0.04204

class CurrentPub 
{
  private:
    ros::Publisher publisher;
 
  public:
    CurrentPub();
    void phidgetsCallback(const cob_phidgets::AnalogSensor& msg);
    void setPub(ros::Publisher publisher);
};

// class functions
CurrentPub::CurrentPub()
{
  // do shit
}
  
void CurrentPub::phidgetsCallback(const cob_phidgets::AnalogSensor& phidgetsMsg)
{
  //ROS_INFO("I heard current 0 = %d", phidgetsMsg.value[0]);
  auckbot_gazebo::MotorCurrents currentMsg;
  currentMsg.time = phidgetsMsg.header.stamp;
  for (int i = 0; i <= 7; i++)
  {  
    if (0 == strcmp(phidgetsMsg.uri[i].c_str(), (char *) "main")) // DC
    {
      currentMsg.main = phidgetsMsg.value[i] / PHIDGET_DC_DIV - PHIDGET_DC_SUB;
    }
    else if (0 == strcmp(phidgetsMsg.uri[i].c_str(), (char *) "motor_1")) // AC
    {
      currentMsg.motor_1 = (float) phidgetsMsg.value[i] * PHIDGET_AC_MUL;
    }
    else if (0 == strcmp(phidgetsMsg.uri[i].c_str(), (char *) "motor_2")) // AC
    {
      currentMsg.motor_2 = (float) phidgetsMsg.value[i] * PHIDGET_AC_MUL;
    }
    else if (0 == strcmp(phidgetsMsg.uri[i].c_str(), (char *) "motor_3")) // AC
    {
      currentMsg.motor_3 = (float) phidgetsMsg.value[i] * PHIDGET_AC_MUL;
    }
    else if (0 == strcmp(phidgetsMsg.uri[i].c_str(), (char *) "motor_4")) // AC
    {
      currentMsg.motor_4 = (float) phidgetsMsg.value[i] * PHIDGET_AC_MUL;
    }
  }
  publisher.publish(currentMsg);
}

void CurrentPub::setPub(ros::Publisher publisher_)
{
  publisher = publisher_;
}

// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "current_pub");
  ros::NodeHandle n;
  
  CurrentPub cb = CurrentPub();
  cb.setPub( n.advertise<auckbot_gazebo::MotorCurrents>("current", 1000) );
  ros::Subscriber sub = n.subscribe("analog_sensors", 1000, &CurrentPub::phidgetsCallback, &cb);
  
  ros::spin();

  return 0;
}

