#!/usr/bin/env bash
# Cancel all tasks and recoveries

echo "Stopping all containers"
docker stop $(docker ps -aq)

if [[ $1 == "--rm" ]]; then
    echo "Removing all containers"
    docker rm $(docker ps -aq)
fi
