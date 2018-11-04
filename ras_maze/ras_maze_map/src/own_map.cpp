/*
 *  world_node.cpp
 *
 *
 *  Created on: Sept 18, 2014
 *  Authors:   Rares Ambrus
 *            raambrus <at> kth.se
 */

/* Copyright (c) 2015, Rares Ambrus, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/cornerList.h>
#include <geometry_msgs/Vector3.h>

// Boost includes
#include <stdio.h>
#include <stdlib.h>



// std includes
#include <limits>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "own_map");
    ros::NodeHandle n("~");
    ros::Rate r(10);


    string _map_file;
    string _map_frame = "/map";
    string _map_topic = "/maze_map";
    n.param<string>("map_file", _map_file, "maze_map.txt");
//    n.param<string>("map_frame", _map_frame, "/map");
//    n.param<string>("map_topic", _map_topic, "/maze_map");

    ROS_INFO_STREAM("Loading the maze map from " << _map_file);
    ROS_INFO_STREAM("The maze map will be published in frame " << _map_frame);
    ROS_INFO_STREAM("The maze map will be published on topic " << _map_topic);

    ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    ros::Publisher wall_coordinates = n.advertise<robo7_msgs::XY_coordinates>("wall_coordinates", 1);
    ros::Publisher corners_coordinates_pub = n.advertise<robo7_msgs::cornerList>("map_corners", 1);
    ros::Publisher walls_coordinates_pub = n.advertise<robo7_msgs::cornerList>("/ras_maze/maze_map/walls_coord_for_icp", 1);

    vector<float> X_wall_coordinates = vector<float>(1, 0);
    vector<float> Y_wall_coordinates = vector<float>(1, 0);
    std::vector<geometry_msgs::Vector3> wall_points;
    float discretisation_step = 0.05;
    int length = 0;

    std::vector<geometry_msgs::Vector3> the_corners_list;
    geometry_msgs::Vector3 corner;

    string line;
    int wall_id = 0;
    while (getline(map_fs, line)){

        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line);

        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line.c_str());
        }

        corner.x = x1;
        corner.y = y1;
        corner.z = 0;
        the_corners_list.push_back(corner);

        corner.x = x2;
        corner.y = y2;
        corner.z = 0;
        the_corners_list.push_back(corner);

        //Discretized map
        int N_step = floor( sqrt(pow(x1-x2,2) + pow(y1-y2,2))/discretisation_step ) + 1;
        float x_step = (x2-x1)/N_step;
        float y_step = (y2-y1)/N_step;
        for(int i=0; i<N_step + 1; i++)
        {
          X_wall_coordinates.push_back(x1 + i*x_step);
          Y_wall_coordinates.push_back(y1 + i*y_step);
          corner.x = x1 + i*x_step;
          corner.y = y1 + i*y_step;
          corner.z = 0;
          wall_points.push_back(corner);
          length ++;
        }
    }

    robo7_msgs::XY_coordinates point_XY;
    point_XY.length = length;
    point_XY.trueX_length = X_wall_coordinates.size();
    point_XY.trueY_length = Y_wall_coordinates.size();
    point_XY.X_coordinates = X_wall_coordinates;
    point_XY.Y_coordinates = Y_wall_coordinates;

    robo7_msgs::cornerList all_corners;
    all_corners.number = the_corners_list.size();
    all_corners.corners = the_corners_list;

    robo7_msgs::cornerList map_points;
    map_points.number = wall_points.size();
    map_points.corners = wall_points;

    // Main loop.
    while (n.ok())
    {
        wall_coordinates.publish( point_XY );
        corners_coordinates_pub.publish( all_corners );
        walls_coordinates_pub.publish( wall_points );
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
