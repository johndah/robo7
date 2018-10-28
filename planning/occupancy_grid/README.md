# Occupancy Grid
call from c++ code with ros::service::call, [explained here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

Service type found in robo7_srvs/IsGridOccupied.srv

(0, 0) is bottom left of the map/grid

## Launch
launch with:
```
roslaunch occupancy_grid occupancy_grid.launch
```
see parameters in launch file

## Manually call service from terminal
```
rosservice call /occupancy_grid/is_occupied "x: 2.0 y: 2.0"
```
where x and y are coordinates in m.

## Visualize grid
The displayed visualization is rotated 90 deg! keep calm and pretend like its
good, the service calls should give the correct results.

Uncomment the two blocks of code in the gauss_filter() function:
```
namedWindow("Display window", cv::WINDOW_NORMAL );
cv::resizeWindow("Display window", 600,600);
imshow( "Display window", grid_in );
cv::waitKey(0);
```
and
```
imshow( "Display window", grid_filtered );
cv::waitKey(0);
cvDestroyWindow("Display window");
```
