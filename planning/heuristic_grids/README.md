# Heuristic Grids
call from c++ code with ros::service::call, [explained here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

Service type found in robo7_srvs/IsGridOccupied.srv
and robo7_srvs/distanceTo.srv

(0, 0) is bottom left of the map/grid

## Launch
launch with:
```
roslaunch heuristic_grids heuristic_grids.launch
```
see parameters in launch file

## Manually call services from terminal
For the occupancy grid:
```
rosservice call /occupancy_grid/is_occupied "x: 2.0 y: 2.0"
```
Or for the distance grid:
```
rosservice call /distance_grid/distance "x_from: 0.22 y_from: 0.22 x_to: 1.5
y_to: 1.5"
```

where coordinates are in m.

## Visualize grids
The displayed visualization is rotated 90 deg! keep calm and pretend like its
good, the service calls give the correct results.

Uncomment the blocks of code regarding the windows:
```
namedWindow("Display window", cv::WINDOW_NORMAL );
cv::resizeWindow("Display window", 600,600);
imshow( "Display window", grid_in );
cv::waitKey(0);
.....
```
