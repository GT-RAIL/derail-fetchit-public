FROM osrf/ros:melodic-desktop-full-bionic

# Set the root directory as HOME
ENV HOME=/root

# Install the apt dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y -q \
        emacs25-nox \
        git \
        libgsl-dev \
        nano \
        python-catkin-tools \
        python-rosinstall-generator \
        ros-melodic-fetch-driver-msgs \
        ros-melodic-sound-play \
        vim && \
    mkdir -p $HOME/ros/stable/src $HOME/ros/active/src

# The commands used to generate the rosinstall file (saved for posterity)
# rosinstall_generator octomap_mapping --deps --exclude RPP > stable.rosinstall

# Clone the repo, initialize & build the workspaces, and then update the
# entrypoint into the container to source the right workspace
WORKDIR $HOME/ros/active
COPY . $HOME/ros/active/src/derail-fetchit
RUN . /opt/ros/melodic/setup.sh && \
    ln -s $HOME/ros/active/src/derail-fetchit/scripts/rosinstall/active.rosinstall $HOME/ros/active/src/.rosinstall && \
    ln -s $HOME/ros/active/src/derail-fetchit/scripts/rosinstall/stable.rosinstall $HOME/ros/stable/src/.rosinstall && \
    cd $HOME/ros/stable/src && \
        wstool up && \
        cd $HOME/ros/stable && \
        CMAKE_PREFIX_PATH=/opt/ros/melodic catkin config --init && \
        catkin build && \
    cd $HOME/ros/active/src && \
        wstool up && \
        cd $HOME/ros/active && \
        CMAKE_PREFIX_PATH=$HOME/ros/stable/devel:/opt/ros/melodic catkin config --init && \
        catkin build && \
    sed -i 's/\/opt\/ros\/\$ROS_DISTRO/\$HOME\/ros\/active\/devel/' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
