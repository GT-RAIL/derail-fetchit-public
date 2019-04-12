#!/usr/bin/env bash

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TAG=${1:-latest}

echo "Running shell railrobotics/derail_fetchit:${TAG}"
docker run -it --rm --network host --privileged -v ${DIR}/../../:/root/ros/active/src railrobotics/derail_fetchit:${TAG} bash
