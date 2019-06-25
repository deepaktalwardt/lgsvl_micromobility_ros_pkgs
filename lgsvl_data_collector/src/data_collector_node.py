#! /usr/bin/env python
import rospy
import os
import numpy as np
import csv
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, CompressedImage
from lgsvl_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, Detection2D, BoundingBox3D, BoundingBox2D
import message_filters
from datetime import datetime

# Save dataset here
folder = "PATH_TO_SAVE_DATASET_AT"
main_camera_subfolder = "main_camera"
depth_camera_subfolder = "depth_camera"

gt_2d_subfolder = "gt_2d/"
gt_3d_subfolder = "gt_3d/"

lidar_pcl_subfolder = "lidar_pcl"

# Annotation Filenames
# This node supports saving annotations in two different formats. See details of formats in DataCollector Class below.
yolo3d_annotations = "gt_2d_yolo3d_annotations.csv"
yolo2_annotations = "gt_2d_yolo2_annotations.csv"
gt_3d_annotations = "gt_3d_annotations.csv"

# Vehicle labels
vehicle_labels = {
    "scooter": "0",
    "hoverboard": "1",
    "skateboard": "2",
    "segway": "3",
    "onewheel": "4"
}

# Number of datapoints saved
datapoint_count = 0

class DataCollector:

    def __init__(self):
        rospy.init_node("data_collector")
        print("YO! The node is running. Now actually collect data")
        self.pcl_pub = rospy.Publisher('/sync_pcl2', PointCloud2, queue_size=5)
        self.listener()
        rospy.spin()

    def callback(self, main_camera, depth_camera, lidar_pcl, gt_2d, gt_3d):
        print("Callback")
        global datapoint_count

        lidar_msg_id = str(lidar_pcl.header.stamp.secs) + str(lidar_pcl.header.stamp.nsecs)[:-3]
        msg_id = str(datetime.now().isoformat())

        if len(gt_2d.detections) > 0:
            self.save_images(main_camera, depth_camera, msg_id)
            self.save_gt_2d(gt_2d, msg_id)
            self.save_gt_3d(gt_3d, lidar_msg_id)
            self.save_lidar(lidar_pcl, lidar_msg_id)
            datapoint_count += 1
        else:
            print("Skipped at: " + msg_id)
        print("Number of datapoints: " + str(datapoint_count))

    def listener(self):

        # Change the topic to apollo camera
        sub_main_camera = message_filters.Subscriber('/apollo/sensor/camera/traffic/image_short/compressed', CompressedImage)
        sub_depth_camera = message_filters.Subscriber('/simulator/sensor/depth_camera/compressed', CompressedImage)
        sub_lidar_pcl = message_filters.Subscriber('/apollo/sensor/velodyne64/compensator/PointCloud2', PointCloud2)
        sub_gt_2d = message_filters.Subscriber('/simulator/ground_truth/mm_2d_detections', Detection2DArray)
        sub_gt_3d = message_filters.Subscriber('/simulator/ground_truth/mm_3d_detections', Detection3DArray)

        ts = message_filters.ApproximateTimeSynchronizer([sub_main_camera, sub_depth_camera, sub_lidar_pcl, sub_gt_2d, sub_gt_3d], 1, 0.1)

        ts.registerCallback(self.callback)
    
    def save_images(self, main_camera, depth_camera, msg_id):

        main_camera_np_arr = np.fromstring(bytes(main_camera.data), np.uint8)
        depth_camera_np_arr = np.fromstring(bytes(depth_camera.data), np.uint8)

        main_camera_cv = cv2.imdecode(main_camera_np_arr, cv2.IMREAD_COLOR)
        depth_camera_cv = cv2.imdecode(depth_camera_np_arr, cv2.IMREAD_GRAYSCALE)

        cv2.imwrite(os.path.join(folder + main_camera_subfolder, 'main-{}.jpg'.format(msg_id)), main_camera_cv)
        cv2.imwrite(os.path.join(folder + depth_camera_subfolder, 'depth-{}.jpg'.format(msg_id)), depth_camera_cv)

    def save_gt_2d(self, gt_2d, msg_id):

        main_image_filename = 'main-{}.jpg'.format(msg_id)
        depth_image_filename = 'depth-{}.jpg'.format(msg_id)

        yolo3d_string = ""
        yolo2_string = ""

        # 2D ground truths for main camera and depth sensor
        for det in gt_2d.detections:
            # YOLO3D style
            x_loc = str(int(det.bbox.x))
            y_loc = str(int(det.bbox.y))
            width = str(int(det.bbox.width))
            height = str(int(det.bbox.height))
            label = vehicle_labels.get(det.label)
            yolo3d_string += x_loc + "," + y_loc + "," + width + "," + height + "," + label + " "

            # YOLOv2 style (x_min=0, x_max=1920, y_min=0, y_max=1080)
            x_min = str(int(max(det.bbox.x - det.bbox.width/2, 0)))
            x_max = str(int(min(det.bbox.x + det.bbox.width/2, 1920))) 
            y_min = str(int(max(det.bbox.y - det.bbox.height/2, 0)))
            y_max = str(int(min(det.bbox.y + det.bbox.height/2, 1080)))
            label = vehicle_labels.get(det.label)
            yolo2_string += x_min + "," + y_min + "," + x_max + "," + y_max + "," + label + " "

        yolo3d_filename = folder + gt_2d_subfolder + yolo3d_annotations
        yolo2_filename = folder + gt_2d_subfolder + yolo2_annotations

        with open(yolo3d_filename, 'a') as f_yolo3d:
            writer = csv.writer(f_yolo3d, delimiter=';')
            writer.writerow([main_image_filename, depth_image_filename, yolo3d_string])
        
        with open(yolo2_filename, 'a') as f_yolo2:
            writer = csv.writer(f_yolo2, delimiter=';')
            writer.writerow([main_image_filename, depth_image_filename, yolo2_string])
    
    def save_gt_3d(self, gt_3d, msg_id):
        # 3D Ground Truth format
        # <filename>;<x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori, x_size, y_size, z_size, label>

        pcd_filename = msg_id + '.pcd'
        obj_string = ""

        for det in gt_3d.detections:
            x_pos = str(det.bbox.position.position.x)
            y_pos = str(det.bbox.position.position.y)
            z_pos = str(det.bbox.position.position.z)
            obj_string += x_pos + "," + y_pos + "," + z_pos + ","

            x_ori = str(det.bbox.position.orientation.x)
            y_ori = str(det.bbox.position.orientation.y)
            z_ori = str(det.bbox.position.orientation.z)
            w_ori = str(det.bbox.position.orientation.w)
            obj_string += x_ori + "," + y_ori + "," + z_ori + "," + w_ori + ","

            x_size = str(det.bbox.size.x)
            y_size = str(det.bbox.size.y)
            z_size = str(det.bbox.size.z)
            obj_string += x_size + "," + y_size + "," + z_size + "," 

            label = vehicle_labels.get(det.label)
            obj_string += label + " "
        
        gt_3d_filename = folder + gt_3d_subfolder + gt_3d_annotations

        with open(gt_3d_filename, 'a') as f_gt_3d:
            writer = csv.writer(f_gt_3d, delimiter=';')
            writer.writerow([pcd_filename, obj_string])

    def save_lidar(self, lidar_pcl, msg_id):
        # Run this cmd in a separate terminal first!
        # rosrun pcl_ros pointcloud_to_pcd input:=/sync_pcl2 _prefix:="PATH_TO_SAVE_TO" _binary:=false
        self.pcl_pub.publish(lidar_pcl)        

data_collector_node = DataCollector()
