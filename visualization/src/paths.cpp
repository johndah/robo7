#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "paths");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Paths", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::SPHERE;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;
    line_strip.scale.y = 0.01;
    line_strip.scale.z = 0.01;

    // Points are green
    line_strip.color.g = 1.0f;
    line_strip.color.a = 1.0;

    // Line strip is blue
    points.color.b = 1.0;
    points.color.a = 1.0;

 
  
    for (uint32_t i = 0; i < 3; ++i)
    {

        geometry_msgs::Point p;
        p.x = i;
        p.y = i*i;
        p.z = 0;


      points.points.push_back(p);
      line_strip.points.push_back(p);

  
      }



  /*

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      
      .points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }

  */
    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    r.sleep();

    f += 0.04;
  }
}