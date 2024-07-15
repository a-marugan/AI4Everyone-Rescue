## Installation Guide

~~NOTE: MAKE SURE YOU HAVE UNBUNTU 20.04~~
~~NOTE: This installs ROS Noetic~~

NOTE: This is now for **UBUNTU 18.04**
NOTE: This is now for **ROS Melodic**


1. Make sure you have your VM set up. Ubuntu 20.04 is needed. For further help in setting up the VM. 
Please refer to: [Guide on installing Ubuntu VM](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/blob/Team_2/Documents/ProjectDocuments/project-AgentSimulation.md)

2. Go to **http://wiki.ros.org/melodic/Installation/Ubuntu**. and follow instructions. 

## Installing ROS ~~Noetic~~ Melodic

1. In your VM, open the terminal. Then go back to root or main:
```
cd
```

2. Upon completion, set up the sources.lsit by enter the installation code:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
## Error Check for Step 2
**IMPORTANT!!!**

If you get an error similar to `could not get lock/var/lib/dpkg/lock - open`
Then close the terminal, open a new terminal and follow the instructions below:
- Run the commands one by one
```
sudo lsof /var/lib/dpkg/lock
sudo lsof /var/lib/apt/lists/lock
sudo lsof /var/cache/apt/archives/lock
```
- It’s possible that the commands don’t return anything, or return just one number. If they do return at least one number, use the number(s) 
and kill the processes like this (replace the <process_id> with the numbers you got from the above commands):
```
sudo kill -9 <process_id>
```
- You can now safely remove the lock files using the commands below:
```
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock
```
- After that, reconfigure the packages
```
sudo dpkg --configure -a
```
After this, you can retry the installation code from step 2.

## Install Continued
3. Then update the keys for ROS by using entering the following in the terminal:
```
sudo apt install curl # if you haven't already installed curl
```
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/main/ros.asc | sudo apt-key add -
```

4. Set up installation by entering the following:
```
sudo apt update
```

5. Make sure you make a full install to have all features. This might take some time (~10mins you can go take a break for now):
```
sudo apt install ros-melodic-desktop-full
```

6. Set up environment by entering the codes in order (just in case of errors):
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
```
source ~/.bashrc
```

7. Install dependancies for running ROS:
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

8. Initialise rosdep for future use by entering the codes in order (just in case of errors):
```
sudo apt install python-rosdep
```
```
sudo rosdep init
```
```
rosdep update
```



## Running ROS in Ubuntu
NOTE: Extracted from http://wiki.ros.org/ROS/Tutorials


1. In a new terminal, enter:
```
roscore
```

2. In another new terminal, enter the following in order;
```
rosrun turtle
```
This locates the turtle tutorial.
Then locate and enter:
```
rosrun turtlesim
```
This should open a new window with a turtle

3. Open a new terminal and enter:
```
rosrun turtlesim turtle_teleop_key
```

4. To check the data (position, velocity, etc), enter in a new terminal:
```
rosrun rqt_gui rqt_gui
```
Select **Plugins** -> **Topics** -> **Topic Monitor**

5. Returning to the **turtle_teleop_key** terminal. Move the turtle and the data should be updated in real time in the GUI.


## Running Gazebo

1. Open new terminal and enter:
```
gazebo
```
Gazebo 9 should open.


## ROS Tutorials
With ROS and Gazebo installed in your VM, you can now play around with [Tutorials](http://wiki.ros.org/ROS/Tutorials)
