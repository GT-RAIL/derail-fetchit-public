#!/usr/bin/env bash
# Cancel all tasks and recoveries

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TAG=${1:-latest}

echo "Running cancel commands through railrobotics/derail_fetchit:${TAG}"

docker run -d --rm --network host --privileged --name cancel_task \
    railrobotics/derail_fetchit:${TAG} \
    rostopic pub /task_executor/cancel actionlib_msgs/GoalID 'stamp:
    secs: 0
    nsecs: 0
id: ""'

docker run -d --rm --network host --privileged --name cancel_monitor \
    railrobotics/derail_fetchit:${TAG} \
    rostopic pub /task_monitor/cancel actionlib_msgs/GoalID 'stamp:
    secs: 0
    nsecs: 0
id: ""'

docker run -d --rm --network host --privileged --name cancel_recovery \
    railrobotics/derail_fetchit:${TAG} \
    rostopic pub /recovery_executor/cancel actionlib_msgs/GoalID 'stamp:
    secs: 0
    nsecs: 0
id: ""'

echo "Sleeping for 20 sec"
sleep 20

docker stop cancel_task cancel_monitor cancel_recovery
