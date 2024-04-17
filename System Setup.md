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

## Using Github
Follow this guide to learn and practice how to use git! [Guides/Github Guide.md](https://github.com/a-marugan/AI4Everyone-Rescue/blob/main/Guides/GitHub%20Guide.md)



Preamble
This document aims to provide a step-by-step guide for setting up the entire system. The document will contain elements from other guides in the GitLab repository, but the other guides may go into the content in further depth. The scope of the document is as follows.

Installing Ubuntu 18.04 via dual booting locally
Installing Ubuntu 18.04 via Virtual Machine
Installing ROS Melodic
Installing the Exploration ROS Node
Installing the Victim Detection ROS Node


Note
The current victim detection module uses Detectron2 which requires an Nvidia GPU. You can check this on your local machine by opening up device manager, then open up the dropdown menu for display adapters and find the name of your GPU.
If you do have an Nvidia GPU, refer to the next section to install Ubuntu 18.04 via dual booting locally and continue installing the full system
If you don't have an Nvidia GPU, skip over to installing Ubuntu 18.04 via Virtual Machine and continue installing the system with the exploration module only (workarounds for installing the victim detection module are still under development, which includes using YOLOv5 and Google Colab)

Installing Ubuntu 18.04 via dual booting locally
A guide to do this can be found here. Once Ubuntu is setup a few more steps are required for the Nvidia drivers. Run the following commands:

apt search nvidia-driver
sudo apt update
sudo apt upgrade
sudo apt install nvidia-driver-<version> # replace with latest tested proprietary drive version found in https://www.nvidia.com/Download/index.aspx (e.g. sudo apt install nvidia-driver-470)
sudo reboot


After rebooting, to verify everything run:

nvidia-smi



Installing the Virtual Machine
The first step of the guide is to install the virtual machine. Oracle's VirtualBox is highly recommended to do this, since it provides an assortment of features, as well as can run on Windows, Linux, MacOS, and Solaris, among many other operating systems. If you choose to use VirtualBox, then follow the instructions below.

Download the VirtualBox application from LINK.
Install and set up the VirtualBox application.
Download the Ubuntu 18.04 image from LINK; the Ubuntu image should be downloaded in the form of a ZIP folder.
Extract the downloaded Ubuntu ZIP folder, which should contain 3 files.
Double click the file with the .ovf extension; doing so should open up a prompt in the VirtualBox application asking whether you want to import the Ubuntu 18.04 image onto the application.
Press the 'import' button in the prompt and wait.
Launch the Ubuntu 18.04 image by selecting it and pressing 'Start'.


Installing ROS
The next step is to install ROS Melodic, which is a version of ROS that the Virtual Robot League uses. The basic instructions to do this are listed below. For a more comprehensive list of instructions, please go to LINK. For a basic guide on how to use and develop code with ROS, please go to LINK.

Launch the Ubuntu 18.04 image on the VirtualBox application.
Open up a terminal by pressing CTRL + ALT + T.
Run cd ~/ to go to the root directory.
Run the command below to set up the sources.list file for installing ROS Melodic.


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`



Update the ROS keys.


sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/main/ros.asc | sudo apt-key add -



Download the ROS packages from the sources.list file we set up above.


sudo apt-get update



Ensure that a full install of ROS Melodic has been completed.


sudo apt install ros-melodic-desktop-full



Set up the ROS environment.


echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc



Install the dependencies for running ROS Melodic.


sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential



Initialise rosdep for future use.


sudo apt install python-rosdep
sudo rosdep init
rosdep update



Installing the Exploration ROS Node
The developed system comprises of two main components, namely the exploration module and the victim detection module. Thus, the third step of the full system set up is to install the exploration module. The following instructions will cover how to do so. A document with more comprehensive instructions is located at LINK.

In Ubuntu 18.04, open up a terminal by pressing CTRL + ALT + T.
Install the dependencies of the exploration module, specifically for ROS Melodic.


sudo apt-get install ros-melodic-gmapping
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-teb-local-planner
sudo apt-get install python-scikits-learn



Create a catkin workspace and change your directory into it.


source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
sudo apt-get install python3-catkin-tools
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3



Clone the exploration module from this GitLab repository.


git clone https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/tree/main/Resources/Robot%20Exploration



Compile the code.


catkin_make



Installing the Victim Detection ROS Node
As mentioned above, the developed system comprises of the exploration module and the victim detection module. Thus, the fourth step of this guide is to install the victim detection module. The following instructions will cover how to do so, and it is rewritten from https://github.com/DavidFernandezChaves/Detectron2_ros

Go back to root and install the Python Virtual Environment.


cd 
sudo apt-get install python-pip
sudo pip install virtualenv
mkdir ~/.virtualenvs
sudo pip install virtualenvwrapper
export WORKON_HOME=~/.virtualenvs
echo '. /usr/local/bin/virtualenvwrapper.sh' >> ~/.bashrc 



Open up a new terminal by pressing CTRL + ALT + T, and create a virtual environment for Detectron2.


mkvirtualenv --python=python3 detectron2_ros



Within this virtual environment, download all the necessary dependecies.


pip install torch==1.9.0+cu111 torchvision==0.10.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html
pip install cython pyyaml==5.1
pip install -U 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu111/torch1.9/index.html
pip install opencv-python
pip install rospkg



Download the detectron2 packages within the catkin workspace.


cd ~/catkin_ws/src
git clone https://github.com/DavidFernandezChaves/detectron2_ros.git
cd detectron2_ros
git pull --all
git submodule update --init



Open up a new terminal by pressing CTRL + ALT + T, and compile everything.


cd ../..
sudo apt-get install python3-empy
catkin_make 


Some troubleshooting notes are provided in the Troubleshooting section here

Launching the System
Once both the exploration and detection modules have been installed, the system can be launched. This involves launching the individual modules. The instructions below will explain how to do so.

Open up a terminal by pressing CTRL + ALT + T, and run roscore.


roscore



Open up a terminal by pressing CTRL + ALT + T, and launch the environment.


cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash    
export TURTLEBOT3_MODEL=waffle_pi    
roslaunch setup3 maze3.launch 



Open up a second terminal by pressing CTRL + ALT + T, and launch the RRT exploration code.


source ~/catkin_ws/devel/setup.bash   
export TURTLEBOT3_MODEL=waffle_pi
roslaunch rrt_exploration multi_tb3_exploration.launch



Open up a third terminal by pressing CTRL + ALT + T, and launch the Detectron2 code.


workon detectron2_ros
source devel/setup.bash
roslaunch detectron2_ros detectron2_ros.launch



To activate RRT exploration, go to RVIZ and follow the Exploration Process section here

To visualize the Detectron2 output, stay in RVIZ and folow the steps in this video from 39:40 - 41:10 (Note: No detection output will be display as the default node uses a pre-trained model that cannot recognize the victims. We have created a sample trained model for you to quickly overcome this and get a taste of the whole system. Simply download config.yaml and model_final.pth here and follow the instructions in Using Custom Trained Detectron2 Model with Detectron2 Node. To get the best result, create and annotate more victim images in Creating Custom Victim Dataset, and train a Detectron2 model with the dataset in Training Your Own Detectron2 Model)
Open up a fourth terminal by pressing CTRL + ALT + T, and launch the system's API - this acts as the intermediary between the exploration and victim detection modules, and controls them. When launching the API, you can enter 1, 2, or 3 to tell the corresponding robot to stop. Entering 11, 22, 33 will tell the corresponding robot to move.


source ~/catkin_ws/devel/setup.bash   
cd src/rrt_exploration/script
python control.py   
   
