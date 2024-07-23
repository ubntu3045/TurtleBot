# TurtleBot in ROS 2

## 1. Introduction

- The goal for this tutorial: 
    - Simulate TurtleBot in gazebo  
    - Get ideas about how to control physical/simulated TurtleBot
    - Control Turtlebot from keyboard
 




## 2. Preparation

### 2.1. Check packages
Before start, check if there are turtlrbot3* packages

```
Source ROS workspace first

$ source /opt/ros/foxy/setup.bash
```
```
$ ros2 pkg list | grep turtlebot3
```
If you don't have turtlebot3 packages, you can install debian packages or from source code. 
 Install from source code

* Step 1: Download turtlebot3.repos

    ```
    First entering your workspace
    (If you don't have workspace yet, you need to create one with an src folder in it)

    $ wget https://raw.githubusercontent.com/ipa-rwu/\
    turtlebot3/foxy-devel/turtlebot3.repos
    ```
*  Step 2: Using vcstools get packages
  
    Make sure you have "src" folder in you workspace, then run this command to get source code for turtlebot3.
    ```
    $ vcs import src<turtlebot3.repos
    ```

    If you didn't install vcstools, you can install it as followed:

    ```
    sudo sh -c 'echo \
    "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main"\
    > /etc/apt/sources.list.d/ros-latest.list'
    ```

    ```
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net \
    --recv-key 0xAB17C654
    ```

    ```
    sudo apt update && sudo apt install python3-vcstool
    ```

* Step 3: Using rosdep get dependencies
    ```
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src  -y
    ```

* Step 4: Install packages
    ```
    $ colcon build --symlink-install
    ```

* Step 5: Source your workspace
    ```
    $ source install/setup.bash
    ```


## 3. Simulation
In this chapter you will learn how to simulate TurtleBot in gazebo 

0. Open a terminal
   
    If you don't set up ROS Domain ID, then the default ROS_DOMAIN_ID=0.

    In this case, we only work with one turtlebot so we can use default ROS Domain ID. 

    If you want to use different ROS Domain ID, you can perform:
    ```
    $ export ROS_DOMAIN_ID=11
    ```

1. Set up ROS environment arguments
   
   If you use debian packages, 
    ```
    $ source  /opt/ros/foxy/setup.bash
    ```
   
   If you use packages in your workspace:
    
    ```
    First entering your workspace

    $ source  install/setup.bash
    ```

2. Set up turtlebot model

    ```
    $ export TURTLEBOT3_MODEL=burger
    ```

3. Set up Gazebo model path

    ```
    $ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
    prefix turtlebot3_gazebo \
    `/share/turtlebot3_gazebo/models/
    ```

4. Launch Gazebo with simulation world

    ```
    $ ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```
   
![Screenshot from 2024-07-23 20-40-19](https://github.com/user-attachments/assets/72842308-8e56-4bd0-8919-c359b67c1bb7)
with `turtlebot3_house.launch.py` 

![Screenshot from 2024-07-23 20-47-01](https://github.com/user-attachments/assets/f70b611b-97fc-4944-b422-4433080b078f)


# Create a map with gmapping
## Starting TIAGo simulation

cd /tiago_public_ws/
source ./devel/setup.bash

In the first console launch the following simulation 
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true
![Screenshot from 2024-07-24 00-24-38](https://github.com/user-attachments/assets/77369141-8893-4f71-8ee7-8a2c8dac184b)

## Mapping
rosrun key_teleop key_teleop.py

By pressing the arrow keys on this console drive TIAGo around the world. The map being created will be shown. When the world has been fully mapped, as in the below example 
![Screenshot from 2024-07-24 00-26-25](https://github.com/user-attachments/assets/5f6304c1-7346-439a-8750-6cbd0a0ffa73)


