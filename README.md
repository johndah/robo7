# robo7

The autonomous robot has been used in a competition at KTH Royal Institute of Technology testing its ability of 

exploration of walls with a LIDAR and objects to classify with camera,
https://github.com/johndah/robo7/blob/master/robot18-contest-phase1-G7_480p.mp4

rescue, i.e. performing path planning and get to found objects in precious phase, catch them and place them in start posision
https://github.com/johndah/robo7/blob/master/robot18-contest-phase2-G7_480p.mp4

I was responsible for developing ![Exploration](https://github.com/johndah/robo7/blob/master/ExplorationSimulation-2018-12-04_03.56.32%20(Short).mp4) where I used an frontier occupancy grid
and ![Path planning](https://github.com/johndah/robo7/blob/master/PathPlanning-2018-11-15_09.30.11.mp4) where I implemented the hybrid A* algorithm. 

The implementation is based on Robot Operating System and the code is written in C++.

Sample images:
### Path Planning
![Path planning](https://github.com/johndah/robo7/blob/master/pathplanning.png)
### Visualized Occupancy Grid
Whiter means higher probability of being an obstacle
![Occupancy Grid](https://github.com/johndah/robo7/blob/master/OccupancyGrid.png)
### Visualized Distance Grid to target
Whiter means closer to target
![Distance Grid](https://github.com/johndah/robo7/blob/master/DistanceGrid.png)


## Clone base-project files from git
```
cd
git clone https://github.com/KTH-RAS/ras_install.git
cd ras_install/scripts
./install.sh
```

## Launch
Launch main launch file with

```
roslaunch robo7_launch/launch/robo7.launch
```

## Motor
$$
angular Velocity = encoderCount \div CPR  \times 2\pi \times freqency
$$

```
CPR (counts per revolution) = 897.96
frequency = 121
```

range of motor input is -100 ~ 100

- left motor serial number: 469741
positive: forward

- right motor serial number: 469412
negative: forward

encoder output range -30 ~ 30
therefore, estimated angular velocity -25.387 ~ 25.387

- wheel distance : 23.8 - 2.07 = 21.73 cm

- wheel diameter : 9.76 cm
## Connect to NUCs
### Mobile NUC
```
- ssh ras17@192.168.1.202
```
### Stationary NUC
```
- ssh ras27@192.168.1.207
```


## Dead reckoning


## Computer Vision

```c++
// install opencv
sudo apt-get install ros-kinetic-vision-opencv
```
