#!/usr/bin/env bash

set -e

TAG=${1}
NAME=${2}

shift 2
echo "Running railrobotics/derail_fetchit:${TAG} named as ${NAME}"
docker run -it --rm --network host --privileged --name ${NAME} railrobotics/derail_fetchit:${TAG} $@

# Modifications to the above script
# 1. Change -it to -d if you want to run in detached mode
# 2. Remove --rm if you want the stopped container to stick around
# 3. Remove --name ${NAME} if you're OK with the random names that docker
#       assigns
