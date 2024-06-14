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
The instructions here are based on `unitree_mujoco` repo. For more information, please go to: https://github.com/unitreerobotics/unitree_mujoco.

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
Finally, in order to test the Go2 robot, start a new terminal and go to the directory `~/unitree/unitree_mujoco/example/cpp`.
```
cd ~/unitree/unitree_mujoco/example/cpp
```
Make a `build` directory and compile the example file `stand_go2.cpp`.
```
mkdir build && cd build
cmake ..
make -j4
```
Now run this in the `build` directory:
```
./stand_go2
```
This example code starts when you press enter. You will see that your Go2 robot stands up and lies down if successful.

## 6. Debug

- If you cannot run any `./unitree_mujoco` or `./stand_go2` commands:
  - Go to the folder and double-click the executable `unitree_mujoco` to start the MuJoCo.  
![image](https://github.com/a-marugan/AI4Everyone-Rescue/assets/147914534/ec9d7736-9af5-448f-aac2-220a1b9f6eb4)  
  - Then we use a python script to test it. Git clone `unitree_sdk2_python` repo on your root directory `unitree`.
  ```
  cd ~/unitree/
  sudo apt install python3-pip
  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
  cd unitree_sdk2_python
  pip3 install -e .
  ```
  - Then go to directory `~/unitree/unitree_mujoco/example/python` and open the file `stand_go2.py`.
  ```
  cd ~/unitree/unitree_mujoco/example/python
  gedit stand_go2.py
  ```
  - Correct typos in line 32 and line 34. Change name `ChannelFactortyInitialize` to `ChannelFactoryInitialize`.
  - Now run the python script:
  ```
  python3 stand_go2.py
  ```
  - You should see the Go2 robot stands up and lies down.
