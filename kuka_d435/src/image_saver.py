#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import numpy as np
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Instantiate CvBridge
bridge = CvBridge()

def image_callback(data):
    pc = ros_numpy.numpify(data)
    # points=np.zeros((pc.shape[0],3))
    # points[:,0]=pc['x']
    # points[:,1]=pc['y']
    # points[:,2]=pc['z']
    print(type(pc))


def main():
    rospy.init_node('image_saver')
    # Define your image topic
    image_topic = "/camera/depth/color/points"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, PointCloud2, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()