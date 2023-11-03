# Yaskawa ROS1

## Description
This repo allows you to communicate and control the SIA5F Yaskawa robot on ROS1 noetic. Currently, the support for the FS100 controller on ROS2 is not available (refers to [official repo](https://github.com/Yaskawa-Global/motoros2)). If you need to use it with ROS2, you can use the docker container and a ros bridge as described below.

### Using 
If you want to use the packages on your local machine follow the instruction in [local setup](#local-setup), otherwise you can use a docker container as in [docker setup](#docker-setup).

## Local setup
Follow the steps below to set up the package on your local machine. 

1. **Add the repository to your ROS workspace**
    ```bash
    cd ~/my_ros_ws/src
    git clone https://github.com/sara9915/uclv_yaskawa_ros.git
    ```

2. **Install packages dependencies**
   
    The packages listed in `https.rosinstall` have to be installed. You can use `wstool` as follows:
   ```bash
    # In the src of your ros ws
    wstool init #if not already initialized
    wstool merge https://raw.githubusercontent.com/sara9915/uclv_yaskawa_ros/main/https.rosinstall
    wstool update
    ```
   Check dependecies package as follows:
   ```bash
    cd ~/my_ros_ws
    rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
    ```

   Finally, install the following libraries:
   ```bash
    sudo apt-get install libblas-dev liblapack-dev
    ```
   
4. **Build**
    ```bash
    cd ~/my_ros_ws
    catkin build 
    ```


## Docker setup on ROS1
First, create the docker image:
```bash
docker build -t ros_noetic:yaskawa https://raw.githubusercontent.com/sara9915/uclv_yaskawa_ros/main/Dockerfile
```
Then, create the docker container:
```bash
docker run -it --name yaskawa_container ros_noetic:yaskawa
```
Now you are able to launch the ros nodes.

## Docker setup for ROS2 
If you need to use this package in ROS2, you can use the ros_bridge package. First, create the docker image:
```bash
docker build -t ros_noetic:yaskawa https://raw.githubusercontent.com/sara9915/uclv_yaskawa_ros/main/Dockerfile.ros2
```
Then, create the docker container: 
```bash
docker run -it --name yaskawa_container ros_bridge:yaskawa
```
Now, you have a container with ROS1 Noetic, ROS2 Foxy and ros1_bridge installed. Also this repo is build into the container. 
To run the node and establish the communication on ROS2 follows these steps.

1. **Launch the ROS1 node**
   ```bash
   docker run -it --name yaskawa_container ros_bridge:yaskawa
   source /opt/ros/noetic/setup.bash 
   roslaunch sun_yaskawa_nodes bringup_motoman.launch
    ```
2. **Launch the ROS bridge**

   In a new terminal, enter in the previous container:
   ```bash
   docker exec yaskawa_container /bin/bash
    ```
   Then, source the setup.bash to run ros2 commands:
   ```bash
   source /opt/ros/foxy/setup.bash 
    ```
   Now, launch the ros bridge:
   ```bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics 
    ```
    Run ```bash ros2 run ros1_bridge dynamic_bridge --help``` to see the available options for the bridge (ROS1->ROS2, or ROS2->ROS1, or ROS1<->ROS2).



