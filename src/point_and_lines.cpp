#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id = "/map";
    sphere_list.header.stamp = ros::Time::now();
    sphere_list.ns = "points_and_lines";
    sphere_list.action = visualization_msgs::Marker::ADD;

    sphere_list.id = 0;

    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  

    // POINTS markers use x and y scale for width/height respectively
    sphere_list.scale.x = 0.2;
    sphere_list.scale.y = 0.2;

    // Line list is red
    sphere_list.color.g = 1.0f;
    sphere_list.color.a = 1.0;
   
    // Create the vertices for the points and lines
     for (uint32_t i = 1; i < 4; ++i)
     {
       float y = 2 * i;
       float z = 2 * i;

      geometry_msgs::Point p;
      p.x = (int32_t)i - 2;
      p.y = y;
      p.z = z;
     
      sphere_list.points.push_back(p);
       }

    marker_pub.publish(sphere_list);

    r.sleep();
  }
}
