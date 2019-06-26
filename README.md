# ROS Packages to be used with [LGSVL Simulator Micromobility version](https://github.com/deepaktalwardt/lgsvl_simulator_micromobility.git)
This repository contains 4 ROS packages used for various purposes in the project using LGSVL Automotive Simulator. Place these packages into your ROS workspace and run `catkin_make` to build the packages.

## lgsvl_data_collector
This package provides a ROS node for data collection from the LGSVL Simulator. This currently supports collecting camera images, depth sensor images, LiDAR point clouds and 2D, 3D ground truths at synchronized time intervals.

### Annotation formats supported
1. YOLO3D Style : `x, y, width, height, label`
2. YOLOv2/YOLOv3 Style : `x_min. y_min, x_max, y_max, label`

### How to launch
```
roslaunch lgsvl_data_collector data_collector_launcher1.launch
```
To save LiDAR pointclouds, make sure to run `pointcloud_to_pcd` node in the `pcl_ros` package in a separate terminal using the following command
```
rosrun pcl_ros pointcloud_to_pcd input:=/sync_pcl2 _prefix:="PATH_TO_SAVE_TO" _binary:=false  
```

## lgsvl_mm_prediction
This package uses a trained YOLOv3 model to run inferences on incoming camera images from the LGSVL simulator. It outputs the results in two separate ways:
1. In a separate window with detection boxes.
2. Directly into the simulator by publishing to Autoware specific topics.

Trained weights can be downloaded from [here](https://www.dropbox.com/s/a44ly3zd6bzmssw/2d-final-weights-keras-yolo3.h5?dl=0). Add the path of this file to `yolo.py` file to load these weights while running inference.

If you would like to train on your own collected dataset, follow the instructions on this [YOLOv3 repository](https://github.com/deepaktalwardt/keras-yolo3).

### How to launch
```
roslaunch lgsvl_mm_perception mm_2d_perception_launcher.launch
```

## lgsvl_msgs
This package has been provided by [LGSVL](https://github.com/lgsvl) and is cloned from their original repository [here](https://github.com/lgsvl/lgsvl_msgs).

## autoware_msgs
This package provides the messages that can be published to the LGSVL Simulator to display detections directly into the simulator camera window.

## Contributors
* **Deepak Talwar** - (https://github.com/deepaktalwardt)
* **Seung Won Lee** - (https://github.com/swdev1202)
### This repository was originally developed for CMPE 297 - Autonomous Driving course at San Jose State University in Spring 2019.
