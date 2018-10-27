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
Uncomment the following in the main loop (bottom) to get a printout of the grid.
```
	//occupancy_grid_server.printGrid();
```
