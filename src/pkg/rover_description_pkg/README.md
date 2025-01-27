# Rover description package

This package contains the rover URDF files, and also Gazebo classic plugins to use it as a differential drive robot. In future it will be adapted to work in ROS2 and Gazebo Ignition.

The files are organized in the following way:
* **launch**
  * `gazebo.launch`: it starts an empty world of Gazebo and spawns the rover
  * `rviz.launch`: it starts RViz and loads the robot description as a parameter. Visuals, TF and collisions can be visualized here
* **meshes**: contains the meshes used to render the visuals of the rover
* **urdf**
  * `rover_gazebo.xacro`: contains gazebo related tags and macros
  * `rover_macro.xacro`: contains the rover description macros
  * `rover.xacro`: includes all the needed files and calls all the needed macros to include descriptions and Gazebo functionalities
  * `utilities.xacro`: a set of utility macros used mainly for math calculations

To use the Gazebo simulation, you need something that publishes a `cmd_vel` topic of type `geometry_msgs/Twist`.

## Xacro parameters

* `track_width`: width of the rubber tire
* `total_width`: width of the entire rover
* `total_length`: length of the entire rover
* `total_height`: height of the entire rover moving base without sensors turret (from the bottom of the tire to the top of the tire)
* `chassis_width`: width of the internal aluminium frame (without the tires and the axes)
* `chassis_height`: height of the internal aluminium frame (without tires)
* `track_distance`: distance between the middle lines of the tires
* `wheel_axes_distance`: distance between one side's wheels axes
* `rover_z_elev`: how much to elevate the rover chassis frame in order to place its tires flat on the ground
* `collision_z_elev`: how much to elevate the collisions' frames wrt the chassis frame in order to align the collisions with the rover shape
* `wheel_collision_x`: size along x of the collision boxes in the front and rear of the tracks
* `wheel_collision_y`: size along y of the collision boxes in the front and rear of the tracks
* `wheel_collision_z`: size along z of the collision boxes in the front and rear of the tracks

## Issues

* It rocks back and forth when a step signal of linear velocity is given. It is most probably due to the collision boxes or clinders and the fake inertial data. It needs to be resolved before it could be used reliably in Gazebo classic with Gazebo diff drive controller plugin. 
  
  One idea could be using caster wheel spheres to the front and to the rear, also them, only collision, not visual. Or maybe making the track collisions barely touch the ground much more they do now, also updating inertial data, could resolve the issue.
* The STL mesh has to be updated, because there are some flying screws that were not there in CAD, but it's only visual, it should not be a problem for simulation