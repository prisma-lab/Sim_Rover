docker build -t $1 --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .
