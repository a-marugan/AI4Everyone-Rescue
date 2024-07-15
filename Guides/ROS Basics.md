## Introduction

ROS (Robot Operating System) is an open-source collection of software frameworks for robot software development. A robot system usually comprises of a collection of ROS nodes, which exchange information by sending and receiving messages.

* The publisher node is responsible for sending the information.
* The subscriber node is responsible for receiving the information.
* The information is sent through a topic, which acts as an intermediary between the publisher and subscriber nodes. In initialising a topic, one must specify the topic name and message type (e.g., the data type of the information being transmitted).
* The publisher-subscriber relationship is N to M (many to many).

## Basic Usage

To use ROS, you must first install it onto your system. This [LINK](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/tree/main/Guides/Install%20ROS%20and%20Gazebo%20on%20Ubuntu%2020.04) provides instructions to install ROS Melodic (a version of ROS) on the Ubuntu 18.04 system.

### Creating a workspace

Everytime we want to create an application, we must create a workspace. This can be done by creating a directory using `mkdir`, and then telling the system that the directory is a workspace via `catkin_make`. An example is shown below.

```
$ cd ~/
$ mkdir catkin_ws
$ cd ~/catkin_ws
$ catkin_make
```

### Creating packages and nodes

To create packages, we must first change our directory into the source directory of the workspace - note that the source directory is created after the first run of the `catkin_make` command. From there, we can use the `catkin_create_pkg` command and define the package name and dependencies of the package. We can then use the `touch` command to create an empty file within that package (i.e., a ROS node), and use the `chmod` command to assign execute permissions to the created file. An example of this is shown below.

```
$ cd ~/catkin_ws/src/
$ catkin_create_pkg [package_name] [dependency_1] [dependency_2] ...
$ touch [publisher_name]/[subcriber_name]
$ chmod +x [publisher_name]/[subcriber_name]
```

### Compiling and running

To compile the code, we can run the `catkin_make` command in the workspace. We can then use the `source` command to tell ROS where the code for the nodes are. Finally, we can create a launch file (with the `.launch` extension) and run it via `roslaunch`. An example of this is shown below.

```
$ catkin_make
$ source devel/setup.bash
$ roslaunch [package_name] [launch_file].launch
```


## Other Commands

The following is a summary of basic commands to interact with the ROS system.

* The `roslaunch` command can be used to launch ROS programs. The command specifies the ROS package and the file within that package. Packages are collections of scripts, parameters, and configurations relating to some common robot functionality.
* The `roscd` command allows us to navigate to the directory of any ROS package on our system.
* The `rosrun` command can be used to execute the publisher script.
* The `rosnode list` command can be used to list all of the nodes that are currently active on the system, and can be used to verify that the `rosrun` command was successful.
* The `rostopic list` command can be used to list all of the topics that are currently used by the nodes on the system.

## Other Resources

This [LINK](https://clearpathrobotics.com/ros-robot-operating-system-cheat-sheet/) takes you to a website where you can download a ROS cheatsheet.

This [LINK](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/tree/main/Guides/TheConstructSim%20Tutorials) takes you to our summary of TheConstructSim's tutorials, which provides very useful resources to learn how to code in ROS.
