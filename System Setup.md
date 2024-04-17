## Preamble

This document aims to provide a step-by-step guide for setting up the entire system. The document will contain elements from other guides in the GitLab repository, but the other guides may go into the content in further depth. The scope of the document is as follows. 

* Installing Ubuntu 22.04 via dual booting locally
* Installing Ubuntu 22.04 via Virtual Machine
* Installing ROS2
* Using GitHub

### Note 

The current victim detection module uses Detectron2 which requires an Nvidia GPU. You can check this on your local machine by opening up device manager, then open up the dropdown menu for display adapters and find the name of your GPU. 

## Installing Ubuntu 22.04 via dual booting locally
A guide to do this can be found [here](https://www.youtube.com/watch?v=u5QyjHIYwTQ). 
** Note: This video demonstrates the installation of Ubuntu 18.04 however the process remain the same for all future versions as well.
Once Ubuntu is setup a few more steps are required for the Nvidia drivers. Run the following commands:
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
3) Download the Ubuntu 18.04 image from ([Ubuntu Downloads](https://ubuntu.com/download/desktop)) - Download 22.04.4 LTS
4) This should download a single large file onto your computer - this step might take quite a while.
5) From there, we need to set up a Virtual Machine with this Ubuntu files. Many youtube videos and guides can be found online. For convenience a guide has also been linked below.
For a more in depth guide, checkout the following website which navigates through the process step by step with images: [Ubuntu Download Steps - with Images](https://medium.com/@fildzaanf/how-to-install-ubuntu-22-04-1-server-on-virtualbox-fe9ab1e0e8)

## Installing ROS2

The next step is to install ROS2, which is a version of ROS that the Virtual Robot League uses. This process must be done within your new Ubuntu server using For a more comprehensive list of instructions, please go to [ROS2 Setup Guide([ROS2 Download Instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)) - (this is the latest platform available currently but is subject to change).

## Going through ROS tutorials

Congratulations! You've now set up your workspace. That should include your Ubuntu server & corresponding ROS2 platform. From here it's important to familiarise yourself with the platform by going throgh all the beginner and intermediate ROS tutorials to begin with. The tutorials can be found here:
[ROS Tutorials](https://docs.ros.org/en/iron/Tutorials.html)

## Using Github
Follow this guide to learn and practice how to use git! [Guides/Github Guide.md](https://github.com/a-marugan/AI4Everyone-Rescue/blob/main/Guides/GitHub%20Guide.md)



