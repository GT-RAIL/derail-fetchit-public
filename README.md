# derail-fetchit
All RAIL lab code for the 2019 IEEE FetchIt Challenge

## Setup and Installation

The primary setup and installation scripts for this repository live in the [`scripts/`](scripts/) directory. At a high level, we use `rosinstall` files to setup and manage the workspace, and `docker` for deployment.

### Initial Setup

**Make sure that your SSH keys are configured with Github. [Instructions](https://help.github.com/en/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)**

You can either download and run [`scripts/setup_ws.sh`](scripts/setup_ws.sh) in a workspace directory (step 2 below) or follow the following steps:

1. Make sure that you have all the `apt` dependencies that are specified in the `apt-get install` line in the [`setup_ws.sh`](scripts/setup_ws.sh) script.
2. Create a workspace directory on your computer. This can be any directory, such as `$HOME/catkin_ws` or `$HOME/ros` (the latter is the default that Fetch will provide). The rest of these instructions assume your current working directory is `$HOME/ros`.
3. Make two workspaces - `stable` and `active`. According to Fetch, the `stable` workspace is meant for those ROS dependencies that we need but don't actively develop on; `active` is the workspace where our code will primarily live.
```bash
mkdir -p stable/src active/src
```
4. Then clone the `derail-fetchit` repo into the `active` workspace
```bash
git clone git@github.com:gt-rail-internal/derail-fetchit.git active/src/derail-fetchit
```
5. Setup symlinks so that we can use `wstool` to manage the different dependencies and all the other repos
```bash
ln -s $(pwd)/active/src/derail-fetchit/scripts/rosinstall/active.rosinstall active/src/.rosinstall
ln -s $(pwd)/active/src/derail-fetchit/scripts/rosinstall/stable.rosinstall stable/src/.rosinstall
```
6. Initialize the workspaces with `wstool`. This allows us to check the status of the numerous repos that we have easily simply by running `wstool` in either the `active/src` or `stable/src` directories. Note that the `derail-fetchit` repo is not going to be managed by `wstool`.
```bash
cd $HOME/ros/stable/src && wstool up
cd $HOME/ros/active/src && wstool up
```
7. Finally, build the workspaces by making sure to link them.
```bash
source /opt/ros/melodic/setup.bash
cd $HOME/ros/stable && catkin build
source $HOME/ros/stable/devel/setup.bash
cd $HOME/ros/active && catkin build
```

Your workspace should now be ready. All you need to source is `$HOME/ros/active/devel/setup.bash` to run your commands.

### Docker

We will be using docker to deploy our code on the robot. However, docker can also be used to test and develop across multiple machines. Here's a brief introduction to the different commands that you might need to run. Before running any of them, make sure to run the following atleast once:

```bash
docker login
# Username: railrobotics
# Password: <default RAIL password>
```

### Build

To build a docker image with your latest code, run: `script/build_docker.sh [TAG]`. The tag is an optional "branch" name that you can provide to the image.

If you think your docker image should be shared with the rest of us, you can push the image to Docker Hub: `docker push railrobotics/derail_fetchit:<TAG>`. The tag is optional if the image is `latest`. **Do not push the `latest` tag from a feature branch.**

### Run a command

If you want to run a command in the background or test how things will be running on the robot, use the script `script/run_docker.sh <TAG> <CONTAINER_NAME> <commands...>`. The script will start up a container from the image `<TAG>`, assign it the name of `<NAME>`, and then run the `<commands...>`.

Example: `./scripts/run_docker.sh latest derail roslaunch task_executor task_executor.launch`

### Run a shell

You can also mount your current `active` workspace into the docker container and open a shell terminal into the container. This allows you to use an editor on your host operating system to change files, and then run commands in the docker container without having to install the complete workspace on the host OS. To run: `./scripts/run_shell_docker.sh [TAG]`. The tag is again optional.


## Packages

### rail_test_world
Files with RAIL version of test environment for copying to `fetchit_challenge` package.

### manipulation_actions
Standalone manipulation actions and testing for the FetchIt! challenge.

### fetchit_mapping
Code for 2D/3D laser/depth based static localization/collision mapping.

### fetchit_bin_detector
Code for bin pose detection.

### task_executor
The high level task execution package
