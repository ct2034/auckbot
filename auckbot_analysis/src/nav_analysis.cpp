/*************************************************
nav_analysis.cpp                          
-------------------
                                              
Copyright 2014 Christian Henkel
Contact: post@henkelchristian.de 
   
This file is part of Auckbot.

Auckbot is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Auckbot is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Auckbot. If not, see http://www.gnu.org/licenses/.             
***************************************************/
 
#include <string>
#include <exception>

#include <ros/ros.h> 
#include <ros/time.h>  

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>

#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <auckbot_gazebo/MotorCurrents.h>

#include <tf/transform_listener.h>

#define TIME      1
#define THD       0.0001
#define PI        3.14159265359
#define NSECPSEC  1 000 000 000
// Number of metrics float
#define NO_MET_F  4
// Number of metrics string
#define NO_MET_S  1
// String length for name
#define STRL_NAME 20
// String length for description
#define STRL_DESC 200

//MongoDB ----
#define MONGO_HOST  "localhost"
//#define MONGO_DB    "nav_analysis"
#define MONGO_COL   "nav_analysis.trips"

#include <cstdlib>
#include <iostream>
#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"

using namespace mongo;
namespace pt = boost::posix_time;

// global params
ros::NodeHandle* nh;
//-------------

class Metric {
  private:
    char name[STRL_NAME];
    char description[STRL_DESC];
    float value;
    char sValue[STRL_DESC];
    ros::Time timeLast;
    
  public:
    Metric() {}
    Metric(char* _name, char* _description);
    void toString(char*);
    void getName(char* _nam){ strcpy(_nam, name); }
    void getValue(float* _val){ *_val = value; }
    void getValue(char* _val){ strcpy(_val, sValue); }
    void setSValue(char* _val){ strcpy(sValue, _val); }
    void resetValueAndTime();
    void addValue(float _val){ value += _val; }
    float passedTime(ros::Time now);
};

Metric::Metric(char* _name, char* _description) {
  strcpy(name, _name);
  strcpy(description, _description);
  resetValueAndTime();
}

void Metric::resetValueAndTime(){
  value = 0.0;
  strcpy(sValue, "N/A");
  timeLast = ros::Time::now();
}
    
void Metric::toString(char* message) {
  if(strcmp(sValue, "N/A") == 0 & value >= 0.0) { // this is a float Metric
    sprintf(message, "Metric '%s': >%s<.\nValue: %f", name, \
      description, value);
  }
  else if (strcmp(sValue, "N/A") != 0 & value == 0.0) { // this is a String Metric
    sprintf(message, "Metric '%s': >%s<.\nValue: %s", name, \
      description, sValue);
  } else {
    ROS_ERROR("String and value set for Metric '%s'", name);
  }
  return;
}

// Returns the time that passed since this was called the last time. (in seconds)
float Metric::passedTime(ros::Time now) {
  float sec = now.toSec() - timeLast.toSec();
  timeLast = ros::Time( now );
  return sec;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class metricListener {
  private:
    tf::Transform oldPose;
    Metric metrics[NO_MET_F+NO_MET_S];
    DBClientConnection* c;
    BSONObjBuilder* b;
    BSONArrayBuilder* ab[2]; // 0: covered_route, 1: planned_route
    bool builderCreated;
    bool debug;
    bool initPoseSet;
    pt::ptime* startTime;
    geometry_msgs::Pose goal;
    tf::Transform initp;
   
  public:
  //constructors
    metricListener();
    metricListener(bool debug);
  //setter / getter
    void addPoint(tf::StampedTransform transform);
    void setOldPose(tf::StampedTransform transform){ \
      oldPose = tf::StampedTransform( transform ); }
  //callbacks  
    void resultCallback(const move_base_msgs::MoveBaseActionResult msg);
    void goalCallback(const move_base_msgs::MoveBaseActionGoal msg);
    void currentsCallback(const auckbot_gazebo::MotorCurrents msg);
  //DB access
    void saveToDB(char* name, char* value);
    void saveToDB(char* name, float value);
    void pointToDBArr(int n, float x, float y, float th);
    void finalize(void);
  //helper
    long long int convertTime(pt::ptime time);
    void checkCreation(void);
    void poseToXYTh(geometry_msgs::Pose pose, float coords[]);
    void poseToXYTh(tf::Transform transform, float coords[]);
    void getPar(char* name, std::string* val);
};

// class functions
//constructors
metricListener::metricListener() {
}

metricListener::metricListener(bool _debug) {
  metrics[0] = Metric((char*) "Length [m]", \
    (char*) "The total length of the route");
  metrics[1] = Metric((char*) "Rotation [rad]", \
    (char*) "The total rotation along the route");
  metrics[2] = Metric((char*) "Current [As]", \
    (char*) "Consumed motor current");
  metrics[3] = Metric((char*) "Duration [ms]", \
    (char*) "Total time used");
  metrics[4] = Metric((char*) "Planner Setup", \
    (char*) "Values that are set for setup of move_base");
  oldPose = tf::Transform();

  try{
    c = new DBClientConnection();
    c->connect(MONGO_HOST);

  } catch( const mongo::DBException &e ) {
    ROS_ERROR("caught %s", e.what());
  }

  builderCreated = false;
  initPoseSet = false;
  debug = _debug;
}

void metricListener::addPoint(tf::StampedTransform transform) {
  float newp[3];
  poseToXYTh(transform, newp);
  float oldp[3];
  poseToXYTh(oldPose, oldp);
   
  float dist = sqrt( pow(newp[0] - oldp[0], 2) +
                     pow(newp[1] - oldp[1], 2) );
  float rot = fabs( newp[2] - oldp[2] );
  if (rot > PI) rot -= 2 * PI;
  rot = fabs(rot);

  if (dist>THD) 
  {
    metrics[0].addValue(dist);
    metrics[1].addValue(rot);

    pointToDBArr(0, newp[0], newp[1], newp[2]); // adding to covered_route (0)

    this->setOldPose(transform);
  }
  if (!initPoseSet) {
    initp = transform;
    initPoseSet = true;
  }
}

// callbacks
void metricListener::resultCallback(const move_base_msgs::MoveBaseActionResult msg) {
  // ROS_INFO("1");
  float check;
  metrics[0].getValue(&check); // evaluate to filter out to short trips

  // capture duration
  pt::time_duration dura = ros::Time::now().toBoost() - *startTime;
  float durationMillis = dura.total_milliseconds();
  metrics[3].addValue(durationMillis);

  // ROS_INFO("2");
  if(!debug & check>0.0) { // save to DB
    char name[STRL_NAME];
    char des[STRL_DESC];
    float val;
    // ROS_INFO("3");
    for(int i=0; i<NO_MET_F; i++) {
      // ROS_INFO("3.1");
      metrics[i].getName(name);
      metrics[i].getValue(&val);
      // ROS_INFO("3.2");   
      saveToDB((char *) name, val);  
      // ROS_INFO("3.3");    
      // saveToDB((char *) "name", 0.1); 
    }
    // ROS_INFO("4");
    for(int i=NO_MET_F; i<NO_MET_F+NO_MET_S; i++) {
      metrics[i].getName(name);
      metrics[i].getValue(des);
      saveToDB((char *) name, (char *) des);
      // saveToDB((char *) "name", (char *) "des");      
    }
    // ROS_INFO("5");
    finalize(); 
  } // terminal only

  // ROS_INFO("6");
  if(check == 0.0) ROS_INFO("(This is not saved in the DB ..)");
  
  char stringInfo[200];
  metrics[0].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[1].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[2].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[3].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[4].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[4].~Metric();
  metrics[4] = Metric((char*) "Planner Setup", \
    (char*) "Values that are set for setup of move_base");

  // ROS_INFO("7");
}

void metricListener::goalCallback(const move_base_msgs::MoveBaseActionGoal msg) {
  ROS_INFO("New route ..");
  metrics[0].resetValueAndTime();
  metrics[1].resetValueAndTime();
  metrics[2].resetValueAndTime();
  char params[STRL_DESC];
  
  std::string* par_base_global_planner = new std::string();
  getPar((char *) "/move_base/base_global_planner", par_base_global_planner);
  std::string* par_base_local_planner = new std::string();
  getPar((char *) "/move_base/base_local_planner", par_base_local_planner);
  // std::string* par_use_grid_path = new std::string();
  // getPar((char *) "/move_base/use_grid_path", par_use_grid_path);

  sprintf( params, "base_global_planner: %s \nbase_local_planner: %s \n", \
    par_base_global_planner->c_str(), par_base_local_planner->c_str());
  metrics[4].setSValue(params);
  startTime->~ptime();
  startTime = new pt::ptime(ros::Time::now().toBoost());

  initPoseSet = false;
  goal = msg.goal.target_pose.pose;
}

void metricListener::currentsCallback(const auckbot_gazebo::MotorCurrents msg) {
  float currents = msg.motor_1 + msg.motor_2 + msg.motor_3 + msg.motor_4;
  metrics[2].addValue( currents * metrics[2].passedTime(msg.time) );
}


void metricListener::saveToDB(char* name, char* value) {
  checkCreation();
  b->append(name, value);
}

void metricListener::saveToDB(char* name, float value) {
  checkCreation();
  b->append(name, value);
}

void metricListener::pointToDBArr(int n, float x, float y, float th){
  checkCreation();
  ab[n]->append( BSON_ARRAY( x << y << th ) );
}

void metricListener::checkCreation(void){
  if (!builderCreated) { 
    ab[0] = new BSONArrayBuilder();
    ab[1] = new BSONArrayBuilder();
    b = new BSONObjBuilder();
    builderCreated = true;
    // start time
    b->appendDate("start_time", \
      mongo::Date_t( convertTime( pt::second_clock::local_time() ) )
      ); 
  }
}

void metricListener::finalize(void) {
  try{
    float g[3];
    poseToXYTh(goal, g);
    b->append("Goal [m, m, rad]", BSON_ARRAY( g[0] << g[1] << g[2] )); 
    float i[3];
    poseToXYTh(initp, i);
    b->append("Initial Pose [m, m, rad]", BSON_ARRAY( i[0] << i[1] << i[2] )); 
    b->append("covered_route", ab[0]->arr());
    //b->append("planned_route", ab[1]->arr());

    BSONObj p = b->obj();
    c->insert(MONGO_COL, p);

    b->~BSONObjBuilder();
    ab[0]->~BSONArrayBuilder();
    ab[1]->~BSONArrayBuilder();
    builderCreated = false;
  } catch( const mongo::DBException &e ) {
    ROS_ERROR("caught %s", e.what());
  }
}

// converting a boost time into a mongo time
long long int metricListener::convertTime(pt::ptime time) {
  pt::ptime time_t_epoch(boost::gregorian::date(1970,1,1)); 
  pt::time_duration diff = time - time_t_epoch;
  return diff.total_milliseconds();
}

void metricListener::poseToXYTh(geometry_msgs::Pose pose, float coordsarr[]){
  coordsarr[0] = pose.position.x;
  coordsarr[1] = pose.position.y;
  coordsarr[2] = tf::getYaw(pose.orientation);
}

void metricListener::poseToXYTh(tf::Transform transform, float coordsarr[]){
  coordsarr[0] = transform.getOrigin().x();
  coordsarr[1] = transform.getOrigin().y();
  coordsarr[2] = getYaw(transform.getRotation());
}

void metricListener::getPar(char* name, std::string* val){
  std::string strname(name);
  if(!nh->getParam(strname, *val)) \
    ROS_ERROR("Error getting parameter >%s<", name);
}
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char** argv) { 
  ROS_INFO("Starting node...");
  ros::init(argc, argv, "nav_analysis");
  //ros::NodeHandle nh;
  nh = new ros::NodeHandle();
  ros::Rate rate(TIME);
   
  std::string map_frame;
  std::string robot_frame;
  bool debug;
  if(!ros::param::get("~map_frame", map_frame)) \
    ROS_ERROR("Can not get param map_frame");
  if(!ros::param::get("~robot_frame", robot_frame)) \
    ROS_ERROR("Can not get param robot_frame");
  if(!ros::param::get("~debug", debug)) \
    ROS_ERROR("Can not get param debug");

  metricListener ml = metricListener(debug);
  
  ros::Subscriber resultSub = nh->subscribe("/move_base/result", \
	  1000, &metricListener::resultCallback, &ml);
  ros::Subscriber goalSub = nh->subscribe("/move_base/goal", \
    1000, &metricListener::goalCallback, &ml);
  ros::Subscriber currentSub = nh->subscribe("/current", \
    1000, &metricListener::currentsCallback, &ml);
  tf::TransformListener listener;
  
  ROS_INFO("Waiting for transformation from '%s' to '%s'", \
    map_frame.c_str(), robot_frame.c_str());
  listener.waitForTransform(map_frame, robot_frame, \
    ros::Time::now(), ros::Duration(60.0));
  
  while (nh->ok()){
    tf::StampedTransform transform;
    try {
      listener.lookupTransform(map_frame, robot_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_INFO("%s",ex.what());
    }
    
    ml.addPoint(transform);
    
    ros::spinOnce();
    rate.sleep();
  }

	ROS_INFO("STOPPING NODE");
  return 0;
}
