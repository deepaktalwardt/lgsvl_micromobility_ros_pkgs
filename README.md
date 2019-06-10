# ROS Packages used for CMPE297-Autonomous Driving Spring 2019 project
This repository contains 4 ROS packages used for various purposes in the project using LGSVL Automotive Simulator. Place these packages into your ROS workspace and run `catkin_make` to build the packages.

## lgsvl_msgs
This package has been provided by [LGSVL](https://github.com/lgsvl) and is clone from their original repository [here](https://github.com/lgsvl/lgsvl_msgs).

## autoware_msgs
This package provides the messages that can be published to the LGSVL Simulator to display detections directly into the simulator camera window.

## lgsvl_data_collector
This package provides a ROS node for data collection from the LGSVL Simulator. This currently supports collecting camera images, depth sensor images, LiDAR point clouds and 2D, 3D ground truths at synchronized time intervals.

## lgsvl_mm_prediction
This package uses a trained YOLOv3 model to run inferences on incoming camera images from the LGSVL simulator. It outputs the results in two separate ways (1) in a separate window with detection boxes, (2) directly into the simulator by publishing to Autoware specific topics.
