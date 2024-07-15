# Preamble
This is a starter guide to setup the 2019 Demo Rosject in theConstructSim. It will also include some other features like navigating through the rosject and building upon the custom dataset of images of the virtual victims. 

# Setting up the RosJect
Follow steps 1-7 in https://github.com/RoboCup-RSVRL/RoboCup2021RVRL_Demo/wiki to load up the environment. This just involves steps of making an account, finding the project and running it. Feel free to minimise and move  the terminals around to whatever suits you. 

**Note**: For step 6: 
```
roslaunch rvrl_setup pre1-1_atr.launch
```
You can change which map is accessed by changing the "pre1-1_atr.launch" to something like "pre1-2_atr.launch",  "pre1-3_atr.launch",  "semi-2-atr.launch", "final-1-atr.launch". Full list of possible maps are here: https://github.com/RoboCup-RSVRL/RoboCup2019RVRL_Demo.

# Moving the robots
Open up a new terminal and run: 
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/[ROBOT_NAME]/cmd_vel
```
Where [ROBOT_NAME] is the name of the robot you want to move. To find the name of a robot, click on a robot on the map and its name will be highlighted in the left panel. E.g.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_3/cmd_vel
```
**Note**: A set of movement instructions are given after the code is executed. Run the instructions in the terminal (not in Gazebo) and the robot will start moving.

# Accessing the camera
To access the robot's camera, open up a new terminal and run:
```
rosrun rqt_image_view rqt_image_view
```
Then click on the computer looking icon at the bottom (graphical tools) to open up a new screen. You can resize the window so it fits your liking. You should see a dropdown menu in the top left corner and opening it up shows all the different cameras equipped on each of the robots on the map. 

E.g. `/robot_3/camera/rgb/image_raw` and `/robot_2/camera/thermal/image_raw`

# Adding to the dataset
There are a couple ways to acquire the images on the rosject to add to the dataset.

a) Manually use snipping tool or other screenshotting software (pretty self explanatory)

b) Storing the images and downloading them.
### Storing and Downloading the images
There's a command you can run that will tell the robot to continually take screenshots of the camera until the command is terminated (this can be done by inputting `Ctrl + C` in the same terminal). The terminal will save these images in the current directory you are working in so I recommend after opening up a new terminal, create a folder where you want to store the images i.e.
```
mkdir Images
cd Images
```

The command to run is:
```
rosrun image_view image_saver image:=/[ROBOT_NAME]/camera/rgb/image_raw 
```
where again the [ROBOT_NAME] is the name of the robot, the terminal will start printing out lines saying it has captured images. You can press `Ctrl + C` in the terminal at anytime to stop the running command. 

**Note**: Have the robot move/spin around to acquire multiple different images.

Once this is done, save your rosject by clicking on the project name at the bottom of the screen and selecting `save rosject` on the popup. Then go to the home icon on the bottom right and select `My Rosjects`. This will bring you back to your list of rosjects where the `RVRL 2019 Demo` should be.

Click on the rosject (do not click run) to open up its preview. Near the bottom is a download icon, which you can click to download the rosject to your local PC. It comes in a zipped file and once this is unzipped you should be able to find the folder you stored your images in.

# Annotating the dataset
There are currently two ways to annotate the dataset (feel free to look into other annotation methods!)

a) Box annotation with RoboFlow

b) Instance segmentation annotation with LabelMe

### Box Annotation with RoboFlow
**Note**: The Detectron2 node currently does not support box-annotated datasets. This may be something you can work on.

1.	As a headstart, we have provided you with our own custom box-annotated victim dataset in this directory, with over 200 competition victims annotated. Download and unzip it. 
2.	Create an account on RoboFlow: https://roboflow.com/
3.	After signing in, click ‘Create New Project’. This option should come up under ‘Workspaces’ on the left-hand side. 
4.	When creating a new project - Project name should be the title of the dataset e.g “Rescue Victim Dataset”, License should be CC by 4.0, Project Type should be Object Detection (Bounding Box), and Annotation Group should be “victim”. Then click ‘Create Public Project’.
5.	Now go the ‘Upload’ section on the left-hand side and upload the file from Step 1. 
6.	Click the ‘Finish Uploading’ button in the top right corner. At this stage split the images into roughly 60% train, 20% valid and 10% test if you want. These value can be changed at your own discretion. 
7.	After the current dataset has been uploaded you will then need to upload the extra images you want to add. So again go to the ‘Upload’ section and now upload any new victim images that need annotating. 
8.	Repeat Step 6 again. 
9.	Now the new images should appear under the ‘Unannotated’ heading in the ‘Annotate section’ on the left hand side. 
10.	Click on these images and draw a box around the victim in the image. 
11.	After drawing the box, label it “victim” and save. Go to the next image and repeat until all images have been annotated. 
Note: for negative images – i.e. images without victims in them – can be left unannotated. 
12.	By now, all new images should be automatically added and combined with the original dataset. 
13.	Go to the ‘Dataset’ section on the left-hand side and check that the image sizing is 416 x 416. 
14.	Then click ‘Generate’. 
15.	Change the name to whatever you want the dataset folder heading to be and click Save. 
16.	Click ‘Export’ and make sure the Format is COCO (this may change according the victim detection model that is ultimately chosen). 
17.	Download the zip to computer. 
18.	After you’ve downloaded the new dataset with the images, make sure to unzip it on your computer but then zip/compress **again** before uploading to Teams for other members. 
Note: This extra step may seem unnecessary but we ran into problems when we did not unzip and zip again manually after downloading from RoboFlow. 

### Instance Segmentation Annotation with LabelMe

1. Download the box-annotated victim dataset in this directory, and unzip it.
2. Delete the JSON file in each of the test, train and valid folders.
3. Sort the competition victim images you have taken into the folders from Step 2, with most images in the train folder
4. Follow the instructions [here](https://medium.com/analytics-vidhya/how-to-label-training-image-data-for-instance-segmentation-task-8a8c16fb5a5b) to annotate the images in each folder with instance segmentation.
5. Zip the whole dataset. 
6. To train a Detectron2 model, go to [Training Your Own Detectron2 Model](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/tree/main/Guides/Training%20Your%20Own%20Detectron2%20Model)
