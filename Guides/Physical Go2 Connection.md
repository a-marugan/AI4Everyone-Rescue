# Getting Started on Physical Go2 Robot

This guide is the starting point on how to setup with the physical Go2 robot. Most instructions are originated from the `unitree_ros2` [git repo](https://github.com/unitreerobotics/unitree_ros2.git).

## Prerequisite

- Install Ubuntu 22.04 on your machine
- Setup ROS2 humble

## 1. Setup Unitree ROS2 Environment

In this guide, we would build everything under the root directory `unitree`. You may choose your own root directory.
```
mkdir unitree && cd ~/unitree
```

### 1.1 Build `unitree_sdk2`

In `unitree` directory, git clone the `unitree_sdk2` repo and `cd` into it.
```
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
```
Make a `build` directory and install unitree_sdk2 to your system directory.
```
mkdir build && cd build
cmake ..
sudo make install
```

### 1.2 Build `unitree_ros2`

Go back to the root directory `unitree`.
```
cd ~/unitree
```
First install some dependencies.
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```
Then, git clone the `unitree_ros2` repo and `cd` into it.
```
git clone https://github.com/unitreerobotics/unitree_ros2
cd unitree_ros2
```
Now, before compiling `cyclone-dds`, please ensure that ros2 environment has NOT been sourced when starting the terminal. Otherwise, it may cause errors in compilation.

Check if `source /opt/ros/humble/setup.bash` has been added to the `~/.bashrc` file when installing ROS2, type
```
gedit ~/.bashrc
```
In the file `~/.bashrc`, it needs to be commented out:
```
# source /opt/ros/humble/setup.bash 
```
After that, compile `cyclone-dds`.
```
cd ~/unitree/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds
```
After compiling `cyclone-dds`, ROS2 dependencies is required for compilation of the `unitree_go` and `unitree_api` packages. Therefore, before compiling, it is necessary to source the environment of ROS2.
```
source /opt/ros/humble/setup.bash
```
Finally, compile `unitree_go` and `unitree_api` packages.
```
colcon build
```

## 2. Connect Go2 via Ethernet

### 2.1 Network setup for VirtualBox

If you do not use a virtual machine to run ubuntu, skip this part.

First, we need to bridge the network to VirtualBox. More information could be found on this [link](https://wiki.dave.eu/index.php/VirtualBox_Network_Configuration).

From the VBox main window, select your VM and choose Settings from the toolbar.
![image](https://github.com/user-attachments/assets/62d69b81-fe9d-4a04-a980-dce4159c8e0b)

Go to Network tab, and change the network adapter from 'NAT' to 'Bridged Adapter', then press 'OK'
![image](https://github.com/user-attachments/assets/79ed74b3-25d7-4c2c-9d17-8346f1678bd5)


Then in your VM Settings, go to Network tab, click the icon circled in red. After that, choose 'IPv4', choose 'Manual', type the address as follow and click 'Apply'.
![image](https://github.com/user-attachments/assets/2fcca4ca-e201-4486-8119-0e075f64a265)
Noted that after this step, your VM will not able to access the internet. If you still need access to the internet, do this step last.

Finally, restart your VM. You are now ready to connect to Go2.

### 2.2 Modify `setup.sh` file in `unitree_ros2`

In your terminal, type
```
ip link
```
![image](https://github.com/user-attachments/assets/69718ca3-1ca9-49c2-bdab-3c0195cd03b6)

Remember this network interface. Yours maybe different. You will use your network interface for connecting Go2. In this case is `enp0s3`.

Now, go to your `unitree_ros2` directory.
```
cd ~/unitree/unitree_ros2
```
Modify the `setup.sh` file.
```
gedit setup.sh
```
The file should look like this:
![image](https://github.com/user-attachments/assets/3a67aaaa-2ad6-4b62-9c82-d7693d52b92f)
- Noted that we need to change `foxy` to `humble`.
- Noted that `unitree_ros2` is inside `unitree` root directory.
- Noted that `enp0s3` is the network interface to connect with Go2. Change to the your correct name mentioned above.

### 2.3 Connect and Test

Source the environment to setup the ROS2 support of Unitree robot.
```
source ~/unitree/unitree_ros2/setup.sh
```
If you don't want to source the bash script every time when a new terminal opens, you can write this line into `~/.bashrc`, but attention should be paid when there are multiple ROS environments coexisting in the system.

Connect to the Go2 robot via ethernet cable, and type 
```
ros2 topic list
```
The topic list will look like this if successful.

![image](https://github.com/user-attachments/assets/3e496c64-96ca-4b44-9a4c-a49bdc609245)


Another test would be pinging the Go2 ip address, type
```
ping 192.168.123.161
```
You will receive responses if successful.




