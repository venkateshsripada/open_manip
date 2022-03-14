#!/usr/bin/env python

import freenect
import cv2
import numpy as np
'''
Here the getDepthMap function takes the depth map from the Kinect sensor, clips the array so that the maximum depth is 1023 (effectively removing distance objects and noise) 
and turns it into an 8 bit array (which OpenCV can render as grayscale). The array returned from getDepthMap can be used like a grayscale OpenCV image to demonstrate I 
apply a Gaussian blur. Finally, imshow renders the image in a window and waitKey is there to make sure image updates actually show.
'''


## Grabs a depth map from the Kinect sensor and creates an image from it.

def getDepthMap():	
	depth, timestamp = freenect.sync_get_depth()

	np.clip(depth, 0, 2**10 - 1, depth)
	depth >>= 2
	depth = depth.astype(np.uint8)
	# depthmm = depth.astype(np.float32)
	# print("depth", depth)
	return depth

while True:
	depth = getDepthMap()

	blur = cv2.GaussianBlur(depth, (5, 5), 0)

	cv2.imshow('image', blur)
	cv2.waitKey(10)





# mdev = freenect.open_device(freenect.init(), 0)
# d = freenect.set_depth_mode(mdev, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_REGISTERED)
# print("depth", d)
# freenect.runloop(dev=mdev)