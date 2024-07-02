# Terrain Tool Guide
## Setup
1. First, install dependencies:
```bash
cd unitree_mujoco
pip3 install noise opencv-python numpy
```
2. Open `terrain_generator.py` in the `/unitree_mujoco/terrain_tool` directory and modify the initial configuration at the beginning as below. You may change "scene_VIP" to your preferred xml file name. For the purpose of this guide, we will use it as our scene name.
```python
# Robot directory
ROBOT = "go2"
# Input scene file
INPUT_SCENE_PATH = "./scene.xml"
# Output
OUTPUT_SCENE_PATH = "../unitree_robots/" + ROBOT + "/scene_VIP.xml" # change name here
```
3. Generate xml file:
```bash
cd terrain_tool
python3 ./terrain_generator.py
```
The program will output the terrain scene file to `/unitree_robots/go2/scene_VIP.xml`. 

4. Then, modify the simulator configuration file `/unitree_mujoco/simulate/config.yaml` and set the scene to the newly generated `scene_VIP.xml`:
```yaml
robot_scene: "scene_VIP.xml"
```
If you are using a Python-based simulator, modify `/unitree_mujoco/simulate_python/config.py`:
```python
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene_VIP.xml" 
```
5. After that, run the unitree_mujoco simulator from the `/unitree_mujoco/simulate/build/` directory, and you can see the generated terrain. 
```bash
./unitree_mujoco
```
Alternatively, you may run `simulate` and drag the `scene_VIP.xml` file into the simulator.

![drag and drop](https://github.com/a-marugan/AI4Everyone-Rescue/assets/10972892/ead4eeb2-ad1f-4323-88db-a81049b2d372)


# Function Explanation
To create desired terrain, modify the main function in `terrain_generator.py`. Below is a brief outline of the functions.
##### 1. `AddBox`
Add a cube, parameters:
```python
position=[1.0, 0.0, 0.0] # Center position
euler=[0.0, 0.0, 0.0] # Euler orientation (roll, pitch, yaw) in RADIANS
size=[0.1, 0.1, 0.1] # Size, length x width x height
``` 
##### 2. `AddGeometry`
Add a geometry, parameters:
```python
position=[1.0, 0.0, 0.0] # Center position
euler=[0.0, 0.0, 0.0] # Euler orientation (roll, pitch, yaw) in RADIANS
size=[0.1, 0.1, 0.1] # Size, some geometries only require the first two parameters
geo_type="cylinder" # Geometry type, supports "plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"
``` 
##### 3. `AddStairs`
Add stairs, parameters:
```python
init_pos=[1.0, 0.0, 0.0] # Position of the stair near the ground
yaw=0.0 # Stair orientation
width=0.2 # Stair width
height=0.15 # Stair height
length=1.5 # Stair length
stair_nums=10 # Number of stairs
```
##### 4. `AddSuspendStairs`
Add floating stairs, parameters:
```python
init_pos=[1.0, 0.0, 0.0] # Position of the stair near the ground
yaw=0.0 # Stair orientation
width=0.2 # Stair width
height=0.15 # Stair height
length=1.5 # Stair length
gap=0.1 # Floating gap
stair_nums=10 # Number of stairs
```
##### 5. `AddRoughGround`
Add rough terrain by randomly arranging cubes, parameters:
```python
init_pos=[1.0, 0.0, 0.0] # Position of the first cube
euler=[0.0, -0.0, 0.0], # Terrain orientation relative to the world
nums=[10, 10], # Number of cubes in x and y directions
box_size=[0.5, 0.5, 0.5], # Cube size
box_euler=[0.0, 0.0, 0.0], # Cube orientation
separation=[0.2, 0.2], # Cube separation in x and y directions
box_size_rand=[0.05, 0.05, 0.05], # Random increment of cube size
box_euler_rand=[0.2, 0.2, 0.2], # Random increment of cube orientation
separation_rand=[0.05, 0.05] # Random increment of cube separation
```

##### 6.`AddPerlinHeighField`
Generate terrain based on Perlin noise, parameters:
```python
position=[1.0, 0.0, 0.0],  # Terrain center position
euler=[0.0, 0.0, 0.0],  # Terrain orientation relative to the world
size=[1.0, 1.0],  # Terrain length and width
height_scale=0.2,  # Maximum terrain height
negative_height=0.2,  # Negative height in the z-axis direction
image_width=128,  # Terrain height map image pixel size
image_height=128,
smoothness=100.0,  # Noise smoothness
perlin_octaves=6,  # Perlin noise parameters
perlin_persistence=0.5,
perlin_lacunarity=2.0,
output_heightmap_image="height_field.png"  # Output height map image name
```

##### 7. `AddHeighFieldFromImage`
Generate terrain based on a given image, parameters:
```python
position=[1.0, 0.0, 0.0] # Terrain center position
euler=[0.0, 0.0, 0.0],  # Terrain orientation relative to the world
size=[2.0, 1.6],  # Terrain length and width
height_scale=0.02,  # Maximum terrain height
negative_height=0.1,  # Negative height in the z-axis direction
input_image_path="./unitree_robot.jpeg" # Input image path
output_heightmap_image="height_field.png", # Output height map image name
image_scale=[1.0, 1.0],  # Image scaling factors
invert_grayscale=False # Invert pixel
```

# Euler Orientation Parameters
For **all** functions except for `AddSuspendStairs` and `AddStairs`, the 3 euler orientation parameters correspond to the roll, pitch and yaw respectively. The positive directions for each axis are as follows:

![Roll-Pitch-Yaw](https://github.com/a-marugan/AI4Everyone-Rescue/assets/10972892/c87b6f6a-505a-4ab5-a75b-0312c0da49c0)

As a brief demonstration, see the following code and the corresponding terrain. 

## Code



## Output


