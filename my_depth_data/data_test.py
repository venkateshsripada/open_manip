#!/usr/bin/env python


# def Data_subscriber():

# 	rospy.init_node('data_type', anonymous = True)
# 	rospy.Subscriber('/camera/depth/image', Image, printer)
# 	rospy.spin()

# def printer(info):
# 	print(info) 
# Data_subscriber()

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

def convert_depth_image(ros_image):
    bridge = CvBridge()
    # display_image = np.full(ros_image.data, 255).astype(np.unint8)
    # cv2.imshow('image--', display_image)
    # cv2.waitKey(0)
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)	# [480, 640]
        center_idx = np.array(depth_array.shape) / 2	# [240, 320]
        print ('center depth:', depth_array[center_idx[0], center_idx[1]])

    except CvBridgeError, e:
        print e
     #Convert the depth image to a Numpy array


def pixel2depth():
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw", Image,callback=convert_depth_image, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pixel2depth()
