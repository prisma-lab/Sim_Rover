# rover_bringup

This repository provides launch files and nodes that use ROS Navigation Stack.

## Dependencies
Before use it install the following dependencies:

1 - Navigation stack
```bash
sudo apt-get install ros-<DISTRO>-navigation
  ```

2 - Teb Local Planner
``` bash
sudo apt-get install ros-<DISTRO>-teb-local-planner
```
3 - Move Base
``` bash
sudo apt-get install ros-<DISTRO>-move-base
```

## Use on the rover
Launch first the launch file to bring-up sensors (lidar), motor drives, tf and Gmapping
```bash
roslaunch rover_bringup rover_bringup.launch 
```
This launch file is used also to load all the parameters for the costmap (local and global), Teb local planner and Move Base.
To send a simple goal to the rover use:
```bash
rosrun rover_bringup simple_nav_goal
```
