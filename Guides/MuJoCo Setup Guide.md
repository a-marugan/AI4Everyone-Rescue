# MuJoCo Setup with Go2 Robot
This guide walks you through how to install MuJoCo simulator, and how to import unitree Go2 robot into the simulator.

## Prerequisite
- Install Ubuntu 22.04 on your machine
- Setup ROS2

## 1. Git clone `unitree_mujoco` repo

Choose a directory as your root directory. For this example, we choose `unitree` as our root directory, and `cd` into it.
```
cd ~/unitree
```
Then, git clone `unitree_mujoco` repo.
```
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

## 2. Git clone `unitree_sdk2` repo

Go back to your root directory.
```
cd ~/unitree
```
Then git clone `unitree_sdk2` repo and `cd` into it.
```
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
chmod +x ./install.sh
sudo ./install.sh
```
Make a `build` directory and build the repo.
```
mkdir build && cd build
cmake ..
make
```

## 3. Install MuJoCo

Go back to your root directory.
```
cd ~/unitree
```
First install some dependencies.
```
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```
Then git clone `mujoco` repo and `cd` into it.
```
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco/
```
Make a `build` directory and build the repo.
```
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
Test MuJoCo by running the command in any directory.
```
simulate
```
If the mujoco simulator pops up, the installation is successful.
