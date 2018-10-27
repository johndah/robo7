Ras maze map publisher
======================

This package implements the maze map publisher. The map is read from a file and published as a marker array which can be visualized in rviz. 

## Parameters

The following parameters can be set when starting the node:

* `map_file` - path to the map file (should be stored as ASCII). Default `maze_map.txt`

The TF frame in which the map is published is `/map`

The topic name under which the map is published is `/maze_map`

## Run

To start the node do:

```rosrun ras_maze_map ras_maze_map_node _map_file:=/path/to_file ```

## File format

The map is stored in a text file, 4 numbers per line: `x1 y1 x2 y2`. `x1 y1` specify the start of the segment and `x2 y2` specify the end. Lines which start with a `#` are considered comments and ignored. Lines which contain fewer than 4 number are skipped.

Example map (can also be found in `maps/sample_map.txt`):
```
# walls
0   0   0   4.8
0   4.8 4.8 4.8
4.8 4.8 4.8 0
4.8 0   0   0
# interior
1 0 1 1
2 0 2 3
```

## Visualization

In Rviz, set the fixed frame to the one you specified when starting the node (default is `/map`).

Add a message of type `MarkerArray` and set the topic to the one specified when starting the node (default is `/maze_map`). 

You should be able to view the map in rviz. For the sample map above, it should look something like this:

[Sample map](https://cloud.githubusercontent.com/assets/4798786/9958615/29ea0964-5e0c-11e5-8601-3f1309d0f132.png "Sample map")
