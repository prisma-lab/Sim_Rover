FROM ros:humble

#Uncomment the following line if you get the "release file is not valid yet" error during apt-get
#	(solution from: https://stackoverflow.com/questions/63526272/release-file-is-not-valid-yet-docker)
#RUN echo "Acquire::Check-Valid-Until \"false\";\nAcquire::Check-Date \"false\";" | cat > /etc/apt/apt.conf.d/10no--check-valid-until

RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

#Install essential
RUN apt-get update && apt-get install -y
RUN apt-get install curl -y
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt-get install -y lsb-release gnupg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.listRUN apt-get update
RUN apt-get update
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install software-properties-common dialog apt-utils apt-transport-https -y
RUN apt-get install -y ignition-fortress

RUN apt-get update && apt-get install -y

##You may add additional apt-get here
RUN apt install kmod -y
RUN apt-get install ros-humble-turtlesim -y
RUN apt install ros-humble-rviz2 -y
RUN apt install ros-humble-rqt* -y
RUN apt install minicom -y
RUN apt install screen -y
RUN apt install ros-humble-xacro -y
RUN apt install ros-humble-librealsense2* -y
RUN apt install ros-humble-realsense2-* -y
RUN apt install ros-humble-navigation2 -y
RUN apt install ros-humble-nav2-bringup -y
RUN apt install ros-humble-slam-toolbox -y
RUN apt install ros-humble-turtlebot3-gazebo -y
RUN apt install ros-humble-rmw-cyclonedds-cpp -y
RUN apt install ros-humble-joint-state-publisher-gui -y
RUN apt install ros-humble-rtabmap -y
RUN apt install ros-humble-rtabmap-ros -y
RUN apt install ros-humble-rviz-visual-tools -y
RUN apt install ros-humble-imu-tools -y
RUN apt install ros-humble-octomap-rviz-plugins -y

#JOYSTICK
RUN apt install joystick -y
RUN apt install ros-humble-joy -y
RUN apt install ros-humble-teleop-twist-joy -y


#install for gz sim
RUN apt-get install -y ros-humble-ros-ign-bridge && \
apt-get install -y ros-humble-ros-gz -y
RUN apt-get install ros-humble-controller-manager -y
RUN apt-get install ros-humble-ros2-control -y
RUN apt-get install ros-humble-ros2-controllers -y
RUN apt-get install ros-humble-ign-ros2-control -y
RUN apt-get install ros-humble-ign-ros2-control-demos -y

RUN apt-get install ros-humble-usb-cam -y
RUN apt-get install ros-humble-image-pipeline -y
RUN apt-get install ros-humble-tf-transformations -y

RUN apt-get upgrade -y && apt-get update -y

RUN apt-get install ros-humble-robot-localization -y

RUN apt-get update && apt-get install -y
RUN sudo apt install pip -y
RUN pip3 install opencv-python opencv-contrib-python transforms3d
RUN apt-get update && apt install ros-humble-tf-transformations -y
#RUN apt-get update && apt-get install -y ros-humble-ros-gzharmonic -y

#RUN export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/models
ENV GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/ros2_iiwa/iiwa_description/gazebo/models

#Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME=/home/user
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42

#Add non root user using UID and GID passed as argument
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
RUN echo "user:user" | chpasswd
RUN echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers

RUN sudo adduser user dialout
RUN sudo adduser user video
RUN sudo adduser user sgx
RUN sudo groupadd iio
RUN sudo adduser user iio
RUN sudo adduser user plugdev

USER user

#ROS2 workspace creation and compilation
RUN mkdir -p ${HOME}/ros2_ws/src
WORKDIR ${HOME}/ros2_ws
COPY --chown=user ./src ${HOME}/ros2_ws/src
SHELL ["/bin/bash", "-c"] 
WORKDIR ${HOME}/ros2_ws/src/git

## Clone the required repositories first
#RUN git clone -b humble --single-branch https://github.com/ros-perception/vision_opencv.git
RUN git clone -b humble --single-branch https://github.com/rst-tu-dortmund/costmap_converter.git
RUN git clone -b humble-devel --single-branch https://github.com/rst-tu-dortmund/teb_local_planner.git
## Optional: Clean up unnecessary files if needed
#RUN rm -rf unilidar_sdk/unitree_lidar_ros/src/unitree_lidar_ros


# Return to the workspace root
WORKDIR ${HOME}/ros2_ws

RUN source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep update; rosdep install -i --from-path src --rosdistro humble -y; colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >>  ${HOME}/.bashrc


#Clean image
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user
