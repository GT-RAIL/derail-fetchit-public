FROM osrf/ros:melodic-desktop-full-bionic

# Set the root directory as HOME
ENV HOME=/root

# Install the apt dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y -q \
        emacs25-nox \
        git \
        curl \
        libgsl-dev \
        nano \
        python-catkin-tools \
        python-rosinstall-generator \
        ros-melodic-amcl \
        ros-melodic-effort-controllers \
        ros-melodic-fetch-driver-msgs \
        ros-melodic-map-server \
        ros-melodic-move-base \
        ros-melodic-moveit \
        ros-melodic-moveit-commander \
        ros-melodic-moveit-kinematics \
        ros-melodic-moveit-python \
        ros-melodic-moveit-ros \
        ros-melodic-moveit-ros-control-interface \
        ros-melodic-moveit-ros-visualization \
        ros-melodic-openni2-launch \
        ros-melodic-robot-controllers \
        ros-melodic-slam-karto \
        ros-melodic-sound-play \
        ros-melodic-teleop-twist-keyboard \
        ros-melodic-trac-ik \
        sox \
        vim && \
    curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py && \
    python /tmp/get-pip.py && \
    pip install -U \
        dash \
        joblib \
        matplotlib \
        networkx \
        numpy \
        pandas \
        plotly \
        pydub \
        requests \
        ruamel.yaml \
        sklearn \
        sphinx \
        sphinx-argparse \
        sphinx_rtd_theme \
        treeinterpreter && \
    mkdir -p $HOME/ros/stable/src $HOME/ros/active/src

# The commands used to generate the rosinstall file (saved for posterity)
# rosinstall_generator octomap_mapping --deps --exclude RPP > stable.rosinstall

# Clone the repo, initialize & build the workspaces, and then update the
# entrypoint into the container to source the right workspace
WORKDIR $HOME/ros/active
COPY ./scripts/rosinstall/active.rosinstall $HOME/ros/active/src/.rosinstall
COPY ./scripts/rosinstall/stable.rosinstall $HOME/ros/stable/src/.rosinstall

RUN . /opt/ros/melodic/setup.sh && \
    cd $HOME/ros/stable/src && \
        wstool up && \
        cd $HOME/ros/stable && \
        CMAKE_PREFIX_PATH=/opt/ros/melodic catkin config --init && \
        CMAKE_PREFIX_PATH=/opt/ros/melodic catkin build && \
    cd $HOME/ros/active/src && \
        wstool up && \
        cd $HOME/ros/active && \
        CMAKE_PREFIX_PATH=$HOME/ros/stable/devel:/opt/ros/melodic catkin config --init && \
        CMAKE_PREFIX_PATH=$HOME/ros/stable/devel:/opt/ros/melodic catkin build && \
    sed -i 's/\/opt\/ros\/\$ROS_DISTRO/\$HOME\/ros\/active\/devel/' /ros_entrypoint.sh
# Note that the $HOME variable is hard-coded in the sed command for now

# Actually copy in the repo now in order to speed up builds
COPY . $HOME/ros/active/src/derail-fetchit
RUN . /opt/ros/melodic/setup.sh && \
    cd $HOME/ros/active && \
        CMAKE_PREFIX_PATH=$HOME/ros/stable/devel:/opt/ros/melodic catkin build

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
