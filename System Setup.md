## Preamble

This document aims to provide a step-by-step guide for setting up the entire system. The document will contain elements from other guides in the GitLab repository, but the other guides may go into the content in further depth. The scope of the document is as follows. 

* Installing Ubuntu 18.04 via dual booting locally
* Installing Ubuntu 18.04 via Virtual Machine
* Installing ROS Melodic
* Installing the Exploration ROS Node
* Installing the Victim Detection ROS Node

### Note 

The current victim detection module uses Detectron2 which requires an Nvidia GPU. You can check this on your local machine by opening up device manager, then open up the dropdown menu for display adapters and find the name of your GPU. 

**If you do have an Nvidia GPU**, refer to the next section to install Ubuntu 18.04 via dual booting locally and continue installing the full system

**If you don't have an Nvidia GPU**, skip over to installing Ubuntu 18.04 via Virtual Machine and continue installing the system with the exploration module only (workarounds for installing the victim detection module are still under development, which includes using [YOLOv5](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/tree/main/Resources/Victim%20Detection/YOLOv5) and [Google Colab](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/blob/main/Resources/Victim%20Detection/Detectron2/GoogleColab.md))

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

1) Download the VirtualBox application from [LINK](https://www.virtualbox.org/wiki/Downloads).
2) Install and set up the VirtualBox application.
3) Download the Ubuntu 18.04 image from [LINK](https://www.linuxvmimages.com/images/ubuntu-1804/); the Ubuntu image should be downloaded in the form of a ZIP folder.
4) Extract the downloaded Ubuntu ZIP folder, which should contain 3 files.
5) Double click the file with the `.ovf` extension; doing so should open up a prompt in the VirtualBox application asking whether you want to import the Ubuntu 18.04 image onto the application.
6) Press the 'import' button in the prompt and wait.
7) Launch the Ubuntu 18.04 image by selecting it and pressing 'Start'.

## Installing ROS2

The next step is to install ROS2, which is a version of ROS that the Virtual Robot League uses. The basic instructions to do this are listed below. For a more comprehensive list of instructions, please go to [LINK](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html?fbclid=IwAR1BMtU7T_279HwpSls0bdZFd1uk6QV6IbMuIwyuK_uXMGryi9LjhW40L_8).

## Using Github
This aims to guide new users on how to use github and to have good git-hygiene.

1) When on the main repository 'AI4Everyone-Rescue' click the green '<> Code' button. Make sure you are in the 'Local/HHTTPS', copy link.
2) Navigate to the directory you want this repository to be in. Go into the terminal for that directory:
```
git clone [copied link]
```
3) Make a new branch/ switching to branches:
```
git checkout -b [branch_name]
```
**Make changes to your code accordingly**
4) Add files to branch:
```
git add [filename]
```
5) Commit changes: Once you have added files onto the branch, you must commit and state your changes
```
git commit -m "commit message"
```
Please make sure to make it concise but not ambiguous eg:
```
git commit -m "implemented victim detection system"
```
6) Push your changes:
```
git push 
```
Congratulations! You have now pushed code into the repository and you should be able to see it on GitHub. It is good practice to not merge accept your own code. 
   
