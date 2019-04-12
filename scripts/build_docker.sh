#!/usr/bin/env bash

set -e

echo "Building railrobotics/derail_fetchit:${1:-latest}"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
docker build -t railrobotics/derail_fetchit:${1:-latest} -f ${DIR}/Dockerfile ${DIR}/..
