# ROS Packages to be used with [LGSVL Simulator Micromobility version](https://github.com/deepaktalwardt/lgsvl_simulator_micromobility.git)
This repository contains 4 ROS packages used for various purposes in the project using LGSVL Automotive Simulator. Place these packages into your ROS workspace and run `catkin_make` to build the packages.

## lgsvl_data_collector
This package provides a ROS node for data collection from the LGSVL Simulator. This currently supports collecting camera images, depth sensor images, LiDAR point clouds and 2D, 3D ground truths at synchronized time intervals.

## lgsvl_mm_prediction
This package uses a trained YOLOv3 model to run inferences on incoming camera images from the LGSVL simulator. It outputs the results in two separate ways:
1. In a separate window with detection boxes.
2. Directly into the simulator by publishing to Autoware specific topics.

## lgsvl_msgs
This package has been provided by [LGSVL](https://github.com/lgsvl) and is cloned from their original repository [here](https://github.com/lgsvl/lgsvl_msgs).

## autoware_msgs
This package provides the messages that can be published to the LGSVL Simulator to display detections directly into the simulator camera window.

---
## Collaborators
* **Deepak Talwar** - (https://github.com/deepaktalwardt)
* **Seung Won Lee** - (https://github.com/swdev1202)
### This repository was originally developed for CMPE 297 - Autonomous Driving course at San Jose State University in Spring 2019.
