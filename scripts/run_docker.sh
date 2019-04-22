#!/usr/bin/env bash

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TAG=${1}
NAME=${2}

shift 2
echo "Running railrobotics/derail_fetchit:${TAG} named as ${NAME}"
docker run -it --rm \
    --network host --device /dev/snd --privileged \
    -e ROS_MASTER_URI -e ROS_IP -v /etc/hosts:/etc/hosts \
    -v ${DIR}/../fetchit_mapping/maps:/root/ros/active/src/derail-fetchit/fetchit_mapping/maps \
    -v ${DIR}/../rail_object_recognition/data:/root/ros/active/src/derail-fetchit/rail_object_recognition/data \
    -v ${DIR}/../task_executor/task_executor:/root/ros/active/src/derail-fetchit/task_executor/task_executor \
    --name ${NAME} \
    railrobotics/derail_fetchit:${TAG} \
    $@

# The volume argment ensures that maps are shared across containers
# Modifications to the above script
# 1. Change -it to -d if you want to run in detached mode
# 2. Remove --rm if you want the stopped container to stick around
# 3. Remove --name ${NAME} if you're OK with the random names that docker
#       assigns
# 4. Change to -v ${DIR}/../:/root/ros/active/src/derail-fetchit if you want to
#       mount your current working code into the container
