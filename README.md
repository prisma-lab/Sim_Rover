<!-- GETTING STARTED -->
## Overview
- This package is being created to add necessary features and improvements for our robots, specifically for ros2. This package is a ros2 porting of a indoor navigation project and implement integration with Slam and Navigation ROS2 package.

  

- This package is exclusively built for ROS2. It is being tested on Ubuntu 20.04 with ROS2-Humble.

  

  

- All the branches of this package are relative to a specific Sensor integration.

<div align="center">
    <img src="docs/real_rover.png" width="75%"/>
</div>

## Table of Contents

1. [Prerequisites](#Prerequisites)
2. [Packages used in Dockerfile](#Packages-used-in-Dockerfile)
3. [Image Compilation and Execution](#Image-Compilation-and-Execution)
4. [Usage](#Usage)
5. [What is launched with this?](#What-is-launched-with-this?)
6. [Navigation2 and Slam Toolbox Configuration](#Navigation2-and-Slam-Toolbox-Configuration)


## Prerequisites
Before setting up the project, you have to install docker. If you already installed docker, go to the next session
```sh
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the Docker packages
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add docker user
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
Then log out and log in.

## Packages used in Dockerfile
- kmod
- minicom
- screen
- xacro
- rviz2
- librealsense2
- realsense2
- navigation2
- nav2-bringup
- slam-toolbox
- rmw-cyclonedds-cpp
- joint-state-publisher-gui
External repositories included in this porject:
- [TEB Local Planner](https://github.com/rst-tu-dortmund/teb_local_planner/tree/ros2-master)
- [Aruco Marker Pose Estimation](https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation)
- [Costmap Converter](https://github.com/rst-tu-dortmund/costmap_converter/tree/ros2/)
## Image Compilation and Execution

1. Clone the repo (complete the command)
```sh
git clone https://github.com/
```
2.  Open a terminal in the repo and source the build file
```sh
source docker_build.sh <IMAGE_NAME>
```
where <IMAGE_NAME> is a name for the image you want to build.

3. In the terminal in the repo, source the run file
```sh
source docker_run.sh <IMAGE_NAME> <CONTAINER_NAME>
```
where <IMAGE_NAME> is the name of the image you have just built, while <CONTAINER_NAME> is a name for the container hosting the image.

### Notes 
1. The container's root password is "user" by default.

2. The container will be automatically destroyed once exited (--rm flag). If you want to attach additional terminals to the container you need to keep it running (docker_run.sh script). You can attach a new terminal by running the following command
```sh
docker exec -it $(docker ps -aqf "name=<CONTAINER_NAME>") bash
```
3. It is recommended to check the correct time for successful image creation 
4. All apt-get performed inside the container will be removed one che container is closed. Please add all new dependacies to the Dockerfile and rebuild the image.
   
## Usage
1. Create a container using this project image on both rover PC an controller PC
2. Set the same ROS domain ID on both rover PCs
```sh
export ROS_DOMAIN_ID=X #number
```
3. Check if the containers times are synchronized, if not use:
 ```sh
sudo date --set="aa-dd-mm h:m:s.ms"
```
4. On the robot terminal source the workspace, compile the project and source the bash.rc:
 ```sh
colcon build --symlink-install
source ./install/setup.bash
```
5. Launch all the nodes on the robot via:
```bash
ros2 launch rover_bringup rover_bringup.launch
```
6. On the controller PC terminal open rviz2

### What is launched with this?
Launch files inside rover_bringup.launch: (1) The Robot Description, (2) The Robot Differential Driver, (3) Sensors Launch, and (4) SLAM and Navigation Launch.
1. The Robot Description: responsible for publishing to the /tf topic and providing transforms between the base_link, base_footprint, and sensor links. Edit the URDF for your robot to define new frames or remove links
2.  The Robot Differential Driver: motor controller driver, responsible for interfacing with the robot and handling velocity commands and publish wheel odometry
3.  Sensors Launch: sensor particular launch file. 
4.  SLAM and Navigation Launch: files to start the slam_toolbox to actuate SLAM and the nav2 pkg for the navigation stack, both files take as input a .yaml configuration file to setup the parameters.


All this launch files are available separately in the launch folder of the rover_bringup package.

## Navigation2 and Slam Toolbox Configuration
It has been provided launch files and configs for Navigation2 and Slam Toolbox. They are available in the ``rover_bringup`` package.

They can be launched with the following launch commands:
```
ros2 launch rover_bringup online_async_launch.py
ros2 launch rover_bringup  navigation_launch.py
```
To change the SLAM and Navigation parameters, work on:
- mapper_params_online_async file in the confif folder.
- nav2_params file in the params folder. 

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

