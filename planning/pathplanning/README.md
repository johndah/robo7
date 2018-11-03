# Path planning

## Run
Run with 
```
roslaunch robo7_launch robo7.launch
```
to launch the nodes
```
<node pkg="pathplanning" type="pathplanning" name="pathplanning" output="screen"/>
<node pkg="visualization" type="paths" name="paths" output = "screen"/> 
```

that depends on launching 

```
node pkg="dead_reckoning" type="dead_reckoning" name="dead_reckoning">/>
<node pkg="occupancy_grid" type="occupancy_grid_server" name="occupancy_grid_server" output = "screen"/>
``` 

## Subscribe to extracted trajectory

For the pathfollower, the pathplanning node publishes through the topic ```/pathplanning/trajectory``` with the messages 

```
robo7_msgs::trajectory trajectory_msg;
robo7_msgs::trajectory_point trajectory_point_msg;
```
where the latter includes 
```
int64 id_number                     // Increasingly id numbers of pathfollowing nodes
geometry_msgs/Vector3 point_coord   // Coordinates point_coord.x, point_coord.y of the nodes
float64 speed                       // Recommended speed based on curvature while following node
float64 distance                    // Distance from node to be followed to previous point, where reaching this implies following next point
```
