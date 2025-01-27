#example call: docker_attach.sh seed_dev_cnt
docker exec -it $(docker ps -aqf "name=$1") bash