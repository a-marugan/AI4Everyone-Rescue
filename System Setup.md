## Preamble

This document aims to provide a step-by-step guide for setting up the entire system. The document will contain elements from other guides in the GitLab repository, but the other guides may go into the content in further depth. The scope of the document is as follows. 

* Installing Ubuntu 18.04 via dual booting locally
* Installing Ubuntu 18.04 via Virtual Machine
* Installing ROS2
* Using GitHub

### Note 

The current victim detection module uses Detectron2 which requires an Nvidia GPU. You can check this on your local machine by opening up device manager, then open up the dropdown menu for display adapters and find the name of your GPU. 

## Installing Ubuntu 18.04 via dual booting locally
A guide to do this can be found [here](https://www.youtube.com/watch?v=u5QyjHIYwTQ). Once Ubuntu is setup a few more steps are required for the Nvidia drivers. Run the following commands:
```
apt search nvidia-driver
sudo apt update
sudo apt upgrade
sudo apt install nvidia-driver-<version> # replace with latest tested proprietary drive version found in https://www.nvidia.com/Download/index.aspx (e.g. sudo apt install nvidia-driver-470)
sudo reboot
```
After rebooting, to verify everything run:
```
nvidia-smi
```

## Installing the Virtual Machine

The first step of the guide is to install the virtual machine. Oracle's VirtualBox is highly recommended to do this, since it provides an assortment of features, as well as can run on Windows, Linux, MacOS, and Solaris, among many other operating systems. If you choose to use VirtualBox, then follow the instructions below.

1) Download the VirtualBox application from [Official VM website](https://www.virtualbox.org/wiki/Downloads).
2) Install and set up the VirtualBox application.
3) Download the Ubuntu 18.04 image from [Ubuntu Downloads](https://www.linuxvmimages.com/images/ubuntu-1804/); the Ubuntu image should be downloaded in the form of a ZIP folder.
4) Extract the downloaded Ubuntu ZIP folder, which should contain 3 files.
5) Double click the file with the `.ovf` extension; doing so should open up a prompt in the VirtualBox application asking whether you want to import the Ubuntu 18.04 image onto the application.
6) Press the 'import' button in the prompt and wait.
7) Launch the Ubuntu 18.04 image by selecting it and pressing 'Start'.

## Installing ROS2

The next step is to install ROS2, which is a version of ROS that the Virtual Robot League uses. The basic instructions to do this are listed below. For a more comprehensive list of instructions, please go to [Ros2 Setup Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html?fbclid=IwAR1BMtU7T_279HwpSls0bdZFd1uk6QV6IbMuIwyuK_uXMGryi9LjhW40L_8).

## Installing ROS

The next step is to install ROS2 - the version to install will , which is a version of ROS that the Virtual Robot League uses. The basic instructions to do this are listed below. For a more comprehensive list of instructions, please go to [LINK](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/tree/main/Guides/Install%20ROS%20and%20Gazebo%20on%20Ubuntu%2020.04). For a basic guide on how to use and develop code with ROS, please go to [LINK](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/blob/main/Guides/ROS%20Basics.md).

1) Launch the Ubuntu 18.04 image on the VirtualBox application.
2) Open up a terminal by pressing `CTRL` + `ALT` + `T`.
3) Run `cd ~/` to go to the root directory.
4) Run the command below to set up the `sources.list` file for installing ROS Melodic.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
```
5) Update the ROS keys.
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/main/ros.asc | sudo apt-key add -
```
6) Download the ROS packages from the `sources.list` file we set up above.
```
sudo apt-get update
```
7) Ensure that a full install of ROS Melodic has been completed.
```
sudo apt install ros-melodic-desktop-full
```
8) Set up the ROS environment.
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
9) Install the dependencies for running ROS Melodic.
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
10) Initialise `rosdep` for future use.
```
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## Going through ROS tutorials

## Using Github
Follow this guide to learn and practice how to use git! [Guides/Github Guide.md](https://github.com/a-marugan/AI4Everyone-Rescue/blob/main/Guides/GitHub%20Guide.md)



