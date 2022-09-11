# Tool to calibrate the 6 DOF between LiDAR and camera using ROS1 and RVIZ.

This repo is forked from [LiDAR_camera_calibration_tool](https://github.com/MarSpit/LiDAR_camera_calibration_tool) and be changed to using **ROS1** to calibrate LiDAR and camera.

Using this tool you can do the calibration between a camera and a LiDAR-sensor. The tool receives the camera image and point cloud from nodes publishing them. Based on the initial calibration parameters the LiDAR points are then projected to the image and an overlay is published and can be shown in RVIZ. Using a GUI, the user can adjust the 6 DOF and thereby create the optimal overlay between LiDAR points and the image. Based on that the calibration matrix is calculated, returned to the terminal and also written to a .txt file. 

![](Images/Tool_in_progress.png)

The tool can either be executed on a system fulfilling the following prerequisites or by building an image from the dockerfile. Both ways are described in detail below.

## Prerequisites 
The code has been developed on **Ubuntu 20.04** in combination with **ROS1 Noetic** and **python 3.8.10**.

## If fulfilling the prerequisites above follow these steps to be able to use the tool:

1. **Download code from this repository and save to desired folder.**

2. **Creating a virtual environment:**
* Creation of a virtual environment in the folder the repository was saved to: `python3 -m venv ~/path_to_tool/venv`
* Activating the virtual environment: `source ~/path_to_venv/venv/bin/activate`

3. **Install necessary python 3 packages from the file *requirements.txt:***
* Enter the following commands in the shell where venv is sourced.
* `cd ~/path_to_tool`
* Installing necessary packages: `pip install -r requirements.txt` (If not working try `pip3 install -r requirements.txt`)
* If a package cannot be found, execute: `pip install --upgrade pip` and try again. (If not working try `pip3 install --upgrade pip`)

4. **Install further necessary packages**
* Run the following commands: 
* `sudo apt-get update`
* `sudo apt-get install ros-ros_distro-cv-bridge` (Replace ros_distro with desired ros1 version)

5. **Adjusting intrinsic parameters:**
* Open *params.py* file and adjust the parameters according to your setup as described below: 
* Enter the initial calibration (*translation*, *rotation*), enter the resolution of the camera images (*width_pixels*, *height_pixels*). The parameter *overlay_publish_timestep* determines how often a overlay is published. Set it to a value  your computer can calculate the overlay in (powerful computer: ~2 sec, not powerful computer: ~6 sec). This time step must be longer than the time step the calibration tool receives the image and point cloud in. Furthermore, adapt the parameters *sub_topic_pcd* and *sub_topic_image* to the name of the topics the image and pcd files are published under.  

1. **Running the calibration tool:**
* Before you start the calibration tool, make sure the ROS1 nodes which are publishing the camera image and point cloud are running. This can be done by opening two new shells and running the image publisher node as well as the pcd publisher node in them. Make sure to source ROS1 before(`source /opt/ros/ros_distro/setup.bash`).
* Go to the shell where the venv is running.
* Make sure ROS1 is sourced. 
* Go to directory where *main.py* file is stored: `cd ~/path_to_tool`.
* Launch the *main.py* file `python3 main.py`. Now the GUI shows up.
* Main.py subscribes to the published point clouds (default topic: */verlodyne_points*) and the published images (default topic: */front*).

7. **Overlay visualization in RVIZ:**
* Open another shell, source ROS1 and run RVIZ: `rviz`
* RVIZ shows up, click the *add* button and add the image topic under which the calibration tool publishes the overlay (default topic: */front_PCD_overlay*).

## Calibration Process

**Creating calibration matrix:**
If you want to adjust the camera LiDAR calibration go to the GUI that just showed up. 
With the upper 3 sliders you can manually adjust the translation and with the lower 3 sliders you can adjust the rotations between LiDAR and camera.
After you moved the sliders to the desired position press the button *Show Overlay* at the GUI. A couple of seconds later the new overlay is shown in RVIZ. Repeat this process until the overlay is clean. After that press the button *Print Calibration Parameters* at the GUI. This will print out the initial calibrations (defined in the params.py file in *translation* and *rotation*), the slider values and the resulting calibration matrix to the terminal. The matrix has the format 4x4. The upper left 3x3 matrix is the rotation matrix and the 4th column is the translation in the format x,y,z,1. Furthermore, these values are written to a file called *calibrations.txt*. The first time you press this button this file will be created, after that new calibrations are appended at the end of the file. 

**Best practice for the calibration process:**
The best results are achieved if you place some kind of calibration object in the FOV of camera and LiDAR. The tests of the tool have been done with a 1x1x1 meter cube. To achieve good results it is recommended to use some kind of 2D or 3D polygon with about the size of the cube. Having 1 meter side length has the advantage, that the magnitude of the offset can be seen quite easily. The tests have been done using a 3D object, which can also be twisted. Nevertheless, a 2D Object should not reduce the calibration accuracy.
If you are using a single cube, a distance of 10 to 20 meters from the sensors is remommended. This is due to have enough points on the cube but to also see the impact of rotation at the same time. To be able to do the calibration even more precisely, another cube can be used. The first one can now be located closer to the sensors and the second one should be located approximately 40 to 60 meters in front of the car. The second cube is especially helpful to improve the rotational calibration, as rotations can be better seen on targets in a larger distance. 

**The difference between rotation and translation**
![](Images/rot_trans.png)

The left picture shows a rotation of 2 degrees around z, while the right picture shows a translation in y of 0.3 m. Rotation makes all points move by a visible amount, while due to perspective transformation the translations of points which are close to the camera is easier to localize than the one from points far away. 

According to experience, 
- Rotation
  - **y** controls yaw axis; 
  - **x** and **z** control roll axis;
- Translation
  - **x** controls left-right;
  - **y** controls up-down;
  - **z** controls zoom in / out the pointcloud;

So you can control rotation-y and translation-x to fix the dislocation of the front and rear point clouds.

## Troubleshooting
* The following error message shows up: 
```
    No image / point cloud has been received. Make sure to run the image and pcd publisher before starting the main.py file and check for correct topic names.The overlay publisher has been stopped.
```

Try / check the following points:
- Make sure the topic names match between the publisher nodes and the tool. This can be checked by opening a new shell, sourcing ROS and running: `rqt_graph`
- Make sure the timestep the camera image and the point cloud are published in is smaller or equal to the timestep the overlay is published in by the tool (this can be adjusted by the parameter *overlay_publish_timestep* in the *params.py* file)
- Stop the calibration tool and the nodes publishing the camera image and the point cloud. Start the publisher nodes, wait a couple of seconds and start the calibration tool again. 
- Check if the ROS message types of the published point cloud and camera image are *Image* and *Pointcloud2*