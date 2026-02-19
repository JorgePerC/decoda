# assignment 

This was made by Jorge Pérez

## Build Instructions

Provided and Ubuntu 20 LTS image and a full installation of ROS Noetic, and this repository is cloned:
```
# Only if you don't have ros installed
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
# Select 1 when prompted 
```

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
# Make sure you have previously sourced the ros environment
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
1. Played rosbag and visualized it on rviz
1. See message data types
    1. After inspecting the point cloud I realized:
        1. The ground was mostly a plane
        1. Obstacles were large, and roughly vertical relative to the ground plane.
1. From experience, I know that debugging ros nodes that try to complete all steps in the same node is pretty hard, so I decided to implement the process pipeline by using the PCL nodelets. 

1. Algorithm selection 
    * Given the sample data, I chose to differentiate obstacles from the ground by their overall slope
    * Since we are working with 3D data, gridding would be required to segment the surface curvature and calculate the normals
    * Instantiate a binary relationship between being an obstacle or ground by the surface normal tilt of each grid space 
    * Normal estimation in PCL uses PCA internally. Since this is implemented in optimized C++, it is acceptable for real-time use in this scenario.

## Questions 

### How to run and monitor the program after start-up

Depends on whether we want manual or automatic execution

|Automatic |Manual|    
|---|---|    
|   If automatically after system boot, we could add a service to systemd to automatically check for physical availability on all sensors, and then run a launch file. On a similar note, rosbags could be stored anywhere on the system, and logrotate could manage their life-cycle, and even compress them to reduce space. | On the other hand, command start-up could be through SSH, whenever the user wants. In my opinion, this would be preferable, since the decisions autonomous systems make can be hard to oversee, a manual startup can be better for safety, and ensures connection to an external host, which can be more convenient since the logs can then be backed-up
|
## Assumptions

* Compute power 
    * Restrained to on-device processing
    * There is no hardware acceleration for the operations (GPU/TPU)
* Robot
    * We are dealing with a ground robot
    * The robot maintains an upright orientation
* Point cloud
    * Reference point to the robot and lidar is fixed
    * Resolution is fixed 
        * 10 Hz
        * mean: 1.45MB min: 1.27MB max: 1.57MB window: 100
* Situation
    * Obstacles are fixed 
    * Ground is relatively flat

## Dependencies

* message_filters
* pcl_ros
* sensor_msgs

While all dependencies used where installed from the default complete ros installation, I had to take into account avoiding scikitlearn, because it conflicts with the ros and numpy python version.


## AI/LLM usage

I used LLM assisted code review and Auto-complete from VS-code. Additionally the GroundSegNode was double checked in order to look for unforeseen bugs. 

## Improvements

### Filtering

Filtering the point cloud by downsizing the sample points could help to reduce noise, and overall reduce execution time by cutting down on the number of elements to process. To be noted, filtering involves data loss, and if an obstacle is relatively small, it might get lost.

I actually ended up doing this by implementing the voxel_grid nodelet, since, after checking the topic frequency, I realized it was only 3 Hz, not so real time. After implementing the change, I saw a frequency of about 9 Hz, close to the /lidar topic publish rate. 

### Complementary algorithms 

We could implement some algorithms from computer vision to improve the segmentation results. Given that this is mostly a morphological segmentation, we could "erode" the ground pointcloud, while at the same time dilating the obstacles to have a clearer view of them. 

On the same note, then automatic clustering could be used by some of the algorithms also available in the PCL, like RANSAC plane fitting.

Even for a resource constrained robot, we could use algorithms like watershed to better find segmentations in the point cloud. A node implementing watershed at a lower frequency could improve accuracy of the ground plane, given that the velocity of the vehicle allows it.

### Better hardware

One of the biggest constraints in a robot is its hardware, from the actuators, to the sensors, to the compute power it has. By implementing SOCs like an Nvidia Jetson, Google Coral, or any other similar hardware accelerator, we can optimize parallel instruction execution, which would in turn allow for AI to be implemented in the decision algorithms, like full feature segmentation and even obstacle recognition.

### Language and code 

* Rewrite the node in C++, it's just faster. 
* My node is not currently accepting parameters to define the tilt degree what differentiates ground from obstacles. Since it's python there are no compilation complaitns, but it is nicer to centralize parameters.

## ROS 2 migration

Given the task to migrate this project, project configuration, node structure and launch files would need to be rewritten. 

While the core perception logic would remain unchanged, migrating to ROS 2 would require architectural updates, including replacing nodelets with composable nodes, rewriting launch files in Python.

## Useful docs:
[Point cloud message structure:](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
[Message Coordinator](https://docs.ros.org/en/api/message_filters/html/python/#message_filters.TimeSynchronizer)
[Efficient Simplification of Point-Sampled Surfaces](https://graphics.stanford.edu/~mapauly/Pdfs/Simplification.pdf)