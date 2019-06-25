#! /usr/bin/env python
import rospy
import os
import numpy as np
from sensor_msgs.msg import CompressedImage
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import Twist
from yolo import YOLO
import cv2
from cv_bridge import CvBridge
from PIL import Image

class MM2DPerception:
    
    # Set apollo to True if the car selected inside the simulator is the Apollo prefab. Set to false if it is from Autoware.
    # Set publish to True if you are using the Autoware prefab and want to visualize detected bounding boxes directly inside the simulator.
    def __init__(self, apollo=True, publish=False):
        self._cv_bridge = CvBridge()
        rospy.init_node('mm_2d_perception_node')
        self.yolo = YOLO()
        self.seq = 0
        self.publish = publish
        self.apollo = apollo

        if self.apollo:
            self.sub_2d = rospy.Subscriber('/apollo/sensor/camera/traffic/image_short/compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self.sub_2d = rospy.Subscriber('/simulator/camera_node/image/compressed', CompressedImage, self.callback, queue_size=1)
        
        if self.publish:
            self.pub_2d = rospy.Publisher('/detection/vision_objects', DetectedObjectArray, queue_size=5)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

    def callback(self, image_msg):
        image_np = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg, "rgb8")
        if not self.publish:
            image_pil = Image.fromarray(image_np) 
            self.infer_image(image_pil)
        else:
            self.get_objects_and_publish(image_np)

    def infer_image(self, image):
        self.seq += 1
        r_image = self.yolo.detect_image(image)

        cv_image = cv2.cvtColor(np.array(r_image), cv2.COLOR_RGB2BGR)

        cv2.imshow("2D Perception", cv_image)
        cv2.waitKey(1)
        # Uncomment the next two lines if you want to save the images as they are inferred
        # r_image.show()
        # r_image.save('PATH_TO_FOLDER/{}.jpg'.format(self.seq))

    def get_objects_and_publish(self, image):
        self.seq += 1
        image_pil = Image.fromarray(image)
        objects = self.yolo.get_detections(image_pil)

        det_obj_arr = DetectedObjectArray()

        for obj in objects:
            det_obj = DetectedObject()

            det_obj.label = obj['label']
            det_obj.score = obj['score']
            det_obj.x = int(obj['x'])
            det_obj.y = int(obj['y'])
            det_obj.width = obj['width']
            det_obj.height = obj['height']

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            det_obj.velocity = twist

            det_obj_arr.objects.append(det_obj)

        self.pub_2d.publish(det_obj_arr)


mm_2d_perception_node = MM2DPerception(apollo=False, publish=True)
