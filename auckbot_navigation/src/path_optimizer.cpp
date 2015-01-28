/*************************************************
path_optimizer.cpp                          
------------------
                                              
Copyright 2014 Christian Henkel
Contact: post@henkelchristian.de 
   
This file is part of Auckbot.

Auckbot is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Auckbot is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Auckbot. If not, see http://www.gnu.org/licenses/.             
***************************************************/

#define DEBUG false

// ros
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"
//#include "tf2/Quaternion.h"

// cgal
#include <CGAL/Cartesian.h>
#include <CGAL/Arr_circle_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

// general
#include <algorithm>    // std::for_each
#include <vector>       // std::vector
#include <memory>       // std::mem_fun
#include <functional>   // std::bind1st

#define CURVE 3
#define FACTOR 1.5

//cgal defs
typedef CGAL::Cartesian<float>                        Kernel;
typedef Kernel::Circle_2                              Circle_2;
typedef CGAL::Arr_circle_segment_traits_2<Kernel>     Traits_2;
typedef Traits_2::CoordNT                             CoordNT;
typedef Traits_2::Point_2                             Point_2;

class PathOptimizer 
{
  private:
    ros::Publisher publisher;
    float min_radius;
    int i, length;
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped curve[CURVE];
    Kernel::Point_2 lastPoint;

  public:
    PathOptimizer(float _min_radius);
    void pathCallback(const nav_msgs::Path& msg);
    void setPub(ros::Publisher publisher);
    void radiusCheck(geometry_msgs::PoseStamped pose);
    void addPose(geometry_msgs::PoseStamped pose);
    void clearPath();

    Kernel::Point_2 poseStamped2Point2(geometry_msgs::PoseStamped pose);
    geometry_msgs::PoseStamped point22PoseStamped(Kernel::Point_2 point, std_msgs::Header h);
};

// class functions
PathOptimizer::PathOptimizer(float _min_radius)
{
  min_radius = _min_radius;
  lastPoint = Kernel::Point_2 (0.0, 0.0);
}
  
void PathOptimizer::pathCallback(const nav_msgs::Path& msg)
{
  length = (int) msg.poses.size();
  if(DEBUG) ROS_INFO("a path, l: %d", length);

  // CALC
  i = 0;
  for_each(msg.poses.begin(), msg.poses.end(), 
    std::bind1st(std::mem_fun(&PathOptimizer::radiusCheck), this));

  if((int) path.size() != length) 
    ROS_ERROR("resulting path has %d instead of %d points",
      length, (int) path.size());

  // SEND
  nav_msgs::Path out_msg;
  out_msg.header = msg.header;
  out_msg.poses = path;

  publisher.publish(out_msg);
  if(DEBUG) ROS_INFO("%s", "published");

  clearPath();
}

void PathOptimizer::setPub(ros::Publisher publisher_)
{
  publisher = publisher_;
}

void PathOptimizer::radiusCheck(geometry_msgs::PoseStamped pose)
{
  curve[i%CURVE] = pose;

  int im2 = i-(CURVE-1);
  int im1 = i-(CURVE-1)/2;

  if (i == 0) // first point
  {
    // nothing to do with this one
    if(DEBUG) ROS_INFO("%s", "Starting radius check");
  }
  else if(i > 0 & i < CURVE) // second point
  {
    // now add first point
    addPose(curve[(i-1)%CURVE]);
  }
  else
  {
    // 1st: edit according to radius,
    try
    {
      Kernel::Point_2 p = poseStamped2Point2(curve[im2%CURVE]);
      Kernel::Point_2 q = poseStamped2Point2(curve[im1%CURVE]);
      Kernel::Point_2 r = poseStamped2Point2(pose);
      
      if(!CGAL::collinear(p ,q ,r ))      
      {
        Circle_2 circ = Circle_2(p, q, r);
        float radius = sqrt(circ.squared_radius());
        // if(DEBUG) ROS_INFO("r: %f", radius);

        if(radius < min_radius)
        {
          Kernel::Vector_2 v = Kernel::Vector_2(circ.center(), q);

          float c = sqrt(CGAL::squared_distance(p, r)) / 2;
          float a = ( sqrt(CGAL::squared_distance(p, q))
                    + sqrt(CGAL::squared_distance(q, r)) ) / 2;
          float d = sqrt( pow(a,2.0) - pow(c,2.0) );          
          float scale = (radius-d/2)/radius;
         
          Kernel::Point_2 q_new = circ.center() + ( scale * v );
          curve[im1%CURVE] = point22PoseStamped(q_new, pose.header);

          if(DEBUG) ROS_INFO("l: %f", radius);
        }  
      }
    }
    catch (int e)
    {
      ROS_ERROR("An exception occured with CGAL");
    }
    
    // then ...
    addPose(curve[im1%CURVE]);
  }

  if(i == (length-1))
  {
    addPose(pose);
  }

  i++;
}

void PathOptimizer::addPose(geometry_msgs::PoseStamped pose)
{
  path.push_back(pose);
}

void PathOptimizer::clearPath()
{
  path.clear();
}

Kernel::Point_2 PathOptimizer::poseStamped2Point2(geometry_msgs::PoseStamped pose)
{
  return Kernel::Point_2 (pose.pose.position.x, pose.pose.position.y);
}

geometry_msgs::PoseStamped PathOptimizer::point22PoseStamped(Kernel::Point_2 point, std_msgs::Header h)
{
  Kernel::Vector_2 dir = Kernel::Vector_2(lastPoint, point);

  geometry_msgs::PoseStamped pose;
  pose.header = h;
  pose.pose.position.x = point.x();
  pose.pose.position.y = point.y();
  pose.pose.position.z = 0.0;
  //pose.pose.orientation = tf2::Quaternion(0, 0, 0);

  lastPoint = point;
  return pose;
}

// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_optimizer");
  ros::NodeHandle n;

  float min_radius;
  if(!ros::param::get("~min_radius", min_radius)) \
    ROS_ERROR("Can not get private param min_radius");
  ROS_INFO("INITIALIZED with r = %.2f", min_radius);
  
  PathOptimizer po = PathOptimizer(min_radius);
  po.setPub( n.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/better_plan", 10) );
  ros::Subscriber   sub = n.subscribe("/move_base/GlobalPlanner/plan", 10,
    &PathOptimizer::pathCallback, &po);
  
  if(DEBUG) ROS_INFO("%s", "...");

  ros::spin();

  return 0;
}

