# Assigment 

This was made by Jorge Pérez

## Build Instructions

Provided the image and a full installation of ROS Noetic:

Set the workspace as your working directory

```
cd decoda_ws
```

Build the workspace 

```
catkin_make
```

Source the project's contents

```
# Make sure to have previously source ros environment
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

```
roslaunch assignment demo.launch
```



## Design overview

### Prior to designing

1. The setup was tested in a Qemu VM
1. Install ros-noetic onto an Ubuntu 20 image
1. Play rosbag and visualice it on rviz
1. See message data types
    1. After seing the point cloud I realiced:
        1. The ground was mostly a plain
        1. Obstacles where big in size, and at a normal positon to the floor
1. From experience, I know that debugging ros nodes that try to complete all steps in the same node is pretty hard, so I decided to split into:

1. Algorithm selection 
    * Given the sample data, I chose to differentiate obstacles from the ground by their overal slope
    * Since we are working with 3D data, gridding would be required to do segment the surface curvature 
    * Instantiate a linear relationship between being an obstacle or ground by the Jacobian of each grid space 
    * PCA, while overall costly to execute, this should INSERT TEXT HERE. It should also be good enough for 



## Questions 

### How to run and monitor the program after start-up

    * Deppends on wheather we want event manual or automatic execution
        * If automatically after system boot, we could add a service to systemd to automatically check for physical availability on all sensors, and then run a launchfile.
        On a similar note, rosbags could be stored anywhere on the system, and logrotate could manage their life-cycle, and even compress them to reduce space. 
        * On the other hand, at commat start-up could be through SSH, whenever the user wants. In my opition, this would be preferable, since the decisitions autonomous systems make can be hard to oversee, a manual startup can be better for safety, and ensures connection to an external host, which can be more convenient since the logs can then be backed-up

## Assumptions

* Compute power 
    * Restrained to on-device processing
    * There is no hardware accelaration for the operations (GPU/TPU)
 * PCA per point is computationally heavy, so it is discarted because it won't be real time
* Robot
    * We are dealing with a ground robot
    * The robot is upside
    * Reference point to the robot and lidar is fixed 
* Point cloud
    * Reference point in on moving robot
    * Resolution is fixed 
        * 10 Hz
        * mean: 1.45MB min: 1.27MB max: 1.57MB window: 100
* Situation
    * Obstacles are fixed 

## Dependencies

While all depedencies used where installed from the default complete ros installation, I had to take into account avoiding scikitlearn. I had forgotten that the ros numpy and overall system-wide python conflicts witht it 

## AI/LLM usage

Auto-complete is enabled in VS-code, 

## Improvements

### Filtering

Filtering the point cloud by downsizing the sample points could help to reduce noise, and overall reduce exectution time by cutting down on the number of elements to process

### Better hardware

One of the biggest constrains in a robot is it's hardware, from the actuators, to the sensors, to the compute power it has. By implementing SOCs like an Nvidia Jetson, Google Coral, or any other simmilar hardare accelarator, we can optimize parallel instruction execution, which would in term allow for AI to be implemented in the decistion algortims, like full feature segmentation and even obstacle recognition

### Complementary algorithms 
Even for a resource counstrained robot, we could use algorithms like watershed to better find segmentations in the point cloud. A node implementing watershed at a lower frequency could improve accuracy of the ground plane, given that the velocity of the vehicle allows it.

### Language 
Rewrite the node in C++, it's just faster. 

## ROS 2 migration

Given the task to migrate this project, configuration, node structure, launchfiles, dependencies