#!/usr/bin/env bash
# Run everything for the competition

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TAG=${1}
TASK=${2}

shift 2
echo "Running task ${TASK} railrobotics/derail_fetchit:${TAG}"

docker run -d --rm \
    --network host --device /dev/snd --privileged \
    -e ROS_MASTER_URI -e ROS_IP -v /etc/hosts:/etc/hosts \
    -v ${DIR}/../fetchit_mapping/maps:/root/ros/active/src/derail-fetchit/fetchit_mapping/maps \
    -v ${DIR}/../rail_object_recognition/data:/root/ros/active/src/derail-fetchit/rail_object_recognition/data \
    -v ${DIR}/../task_execution/task_executor:/root/ros/active/src/derail-fetchit/task_execution/task_executor \
    --name services \
    railrobotics/derail_fetchit:${TAG} \
    roslaunch task_executor fetchit.launch start_all:=true task_executor:=false

sleep 10

docker run -d --rm \
    --network host --device /dev/snd --privileged \
    -e ROS_MASTER_URI -e ROS_IP -v /etc/hosts:/etc/hosts \
    -v ${DIR}/../fetchit_mapping/maps:/root/ros/active/src/derail-fetchit/fetchit_mapping/maps \
    -v ${DIR}/../rail_object_recognition/data:/root/ros/active/src/derail-fetchit/rail_object_recognition/data \
    -v ${DIR}/../task_execution/task_executor:/root/ros/active/src/derail-fetchit/task_execution/task_executor \
    --name task \
    railrobotics/derail_fetchit:${TAG} \
    roslaunch task_executor fetchit.launch task_executor:=true

sleep 10

docker run -d --rm \
    --network host --device /dev/snd --privileged \
    -e ROS_MASTER_URI -e ROS_IP -v /etc/hosts:/etc/hosts \
    -v ${DIR}/../fetchit_mapping/maps:/root/ros/active/src/derail-fetchit/fetchit_mapping/maps \
    -v ${DIR}/../rail_object_recognition/data:/root/ros/active/src/derail-fetchit/rail_object_recognition/data \
    -v ${DIR}/../task_execution/task_executor:/root/ros/active/src/derail-fetchit/task_execution/task_executor \
    --name run_task \
    railrobotics/derail_fetchit:${TAG} \
    rosrun task_executor run_task.py ${TASK}

docker logs -f task
