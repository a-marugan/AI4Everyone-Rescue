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
The instructions here are based on `unitree_mujoco` repo. For more information, please go to: `https://github.com/unitreerobotics/unitree_mujoco`.

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
  
If this error occur: `Illegal instruction (core dumped)`, and you saw a green turtle on the bottom right corner of your virtual machine,
![image](https://github.com/a-marugan/AI4Everyone-Rescue/assets/147914534/8ca26552-0a07-4488-9685-4a99881c8c06)  
then you need to disable Hyper-V for Windows 11 host by following [this link](https://www.makeuseof.com/windows-11-disable-hyper-v/).

## 4. Compile `unitree_mujoco`

Install `yaml-cpp` for reading configuration files.
```
sudo apt install libyaml-cpp-dev
```
Go to the directory `simulate` inside `unitree_mujoco`.
```
cd ~/unitree/unitree_mujoco/simulate
```
Make a `build` directory and build it.
```
mkdir build && cd build
cmake ..
make -j4
```
## 5. Testing Go2 robot
Try running this command in the `build` directory. The Go2 robot will pop up in the simulation if successful.
```
./unitree_mujoco
```
If the simulation do not pop up, try go through the folder and double-click the executable `unitree_mujoco`.  
![image](https://github.com/a-marugan/AI4Everyone-Rescue/assets/147914534/ec9d7736-9af5-448f-aac2-220a1b9f6eb4)  

Finally, in order to test the Go2 robot, go to the directory `~/unitree/unitree_mujoco/example/cpp`  
and copy the `stand_go2.cpp` file into the directory `~/unitree/unitree_mujoco/simulate/src`.  

Then go back to `simulate` directory add this line to the `CMakeLists.txt`:
```
add_executable(stand_go2 src/stand_go2.cpp)
```
and `cmake` the `simulate` directory again.
```
cd build
cmake ..
make -j4
```
Now open the simulation (`./unitree_mujoco`) and run this in the `build` directory:
```
./stand_go2
```
This example code starts when you press enter. You will see that your Go2 robot stands up and sits down.
