xhost +local:root
IMAGE_ID=$(docker images -q $1)
WORKSPACE_DIR=$(pwd)/src
docker run --rm -it --name=$2 --net=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" --privileged -v /dev:/dev --volume=${WORKSPACE_DIR}/pkg:/home/user/ros2_ws/src/pkg  $1
xhost -local:root
#docker run --rm -it --name=$2 --net=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" --volume=/dev/ttyACM0 --volume=/src:/home/drone/Tesi-Rover/src $1