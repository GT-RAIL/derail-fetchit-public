#!/usr/bin/env bash
# Sets up a workspace in the current directory

set -ex

# Get all the apt dependencies that are necessary
sudo apt-get update
sudo apt-get install -y \
        emacs25-nox \
        git \
        curl \
        libgsl-dev \
        nano \
        python-catkin-tools \
        python-rosinstall-generator \
        ros-melodic-effort-controllers \
        ros-melodic-fetch-driver-msgs \
        ros-melodic-move-base \
        ros-melodic-moveit \
        ros-melodic-moveit-ros \
        ros-melodic-moveit-commander \
        ros-melodic-moveit-ros-visualization \
        ros-melodic-openni2-launch \
        ros-melodic-robot-controllers \
        ros-melodic-sound-play \
        ros-melodic-trac-ik \
        vim
curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
sudo python /tmp/get-pip.py && \
sudo -H pip install -U \
        matplotlib \
        numpy \
        treeinterpreter

# Create the workspace directories
mkdir -p ./stable/src ./active/src

# If the repo has not been cloned, then clone it
if [ ! -d ./active/src/derail-fetchit ]; then
    git clone -b dev git@github.com:gt-rail-internal/derail-fetchit.git ./active/src/derail-fetchit
else
    echo "$(tput bold)Repository exists; Not cloning$(tput sgr0)"
fi

# If the rosinstall files don't exist, setup the symlinks
if [ ! -f ./active/src/.rosinstall ]; then
    ln -s $(pwd)/active/src/derail-fetchit/scripts/rosinstall/active.rosinstall ./active/src/.rosinstall
else
    echo "$(tput bold)Active workspace rosinstall file exists; Not linking$(tput sgr0)"
fi
if [ ! -f ./stable/src/.rosinstall ]; then
    ln -s $(pwd)/active/src/derail-fetchit/scripts/rosinstall/stable.rosinstall ./stable/src/.rosinstall
else
    echo "$(tput bold)Stable workspace rosinstall file exists; Not linking$(tput sgr0)"
fi

# Initialize the workspaces
cd ./stable/src/ && wstool up && cd ../../
cd ./active/src/ && wstool up && cd ../../

# Then build the workspaces
source /opt/ros/melodic/setup.bash
cd ./stable && catkin build && cd ..
source ./stable/devel/setup.bash
cd ./active && catkin build && cd ..
