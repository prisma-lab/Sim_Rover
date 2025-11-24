xhost +local:root
IMAGE_ID=$(docker images -q $1)
WORKSPACE_DIR=$(pwd)/src
docker run --rm -it --name=$2 --net=host -e GZ_PARTITION=shared_world -e GZ_VERBOSE=4  --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" --privileged -v /dev:/dev --volume=${WORKSPACE_DIR}/pkg:/home/user/ros2_ws/src/pkg  $1
xhost -local:root