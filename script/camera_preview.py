#!/usr/bin/env python3

################################################################################
## {Description}: Camera Preview
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class CameraPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False

		# cv2.putText font configuration
		self.fontFace = cv2.FONT_HERSHEY_DUPLEX
		self.fontScale = 0.5
		self.color = (255, 255, 255)
		self.thickness = 1
		self.lineType = cv2.LINE_AA
		self.bottomLeftOrigin = False # if True (text upside down)

		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

		rospy.logwarn("CameraPreview Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to Image msg
		self.image_topic = "/camera_bringup/image_raw"
		self.image_sub = rospy.Subscriber(
											self.image_topic, 
											Image, 
											self.cbImage)

		# Subscribe to CameraInfo msg
		self.cameraInfo_topic = "/camera_bringup/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(
											self.cameraInfo_topic, 
											CameraInfo, 
											self.cbCameraInfo)

		# Allow up to one second to connection
		rospy.sleep(2)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			self.cv_image = cv2.flip(self.cv_image, 1)	# comment if the image is mirrored
			
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False

	# Get CameraInfo
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height

	# Information on output screen
	def cbInfo(self):
	
		cv2.putText(
				self.cv_image, 
				"{}".format(self.timestr), 
				(10, 20), 
				self.fontFace, 
				self.fontScale, 
				self.color, 
				self.thickness, 
				self.lineType, 
				self.bottomLeftOrigin
				)

		cv2.putText(
				self.cv_image, 
				"Sample", 
				(10, self.imgHeight-10), 
				self.fontFace, 
				self.fontScale, 
				self.color, 
				self.thickness, 
				self.lineType, 
				self.bottomLeftOrigin
				)

		cv2.putText(
				self.cv_image, 
				"(%d, %d)" % (self.imgWidth, self.imgHeight), 
				(self.imgWidth-100, self.imgHeight-10), 
								self.fontFace, 
				self.fontScale, 
				self.color, 
				self.thickness, 
				self.lineType, 
				self.bottomLeftOrigin
				)

	# Show the output frame
	def cbShowImage(self):

		cv2.imshow("CameraPreview", self.cv_image)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
			self.cbInfo()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("CameraPreview Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_preview', anonymous=False)
	camera = CameraPreview()
	
	r = rospy.Rate(60)

	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
		r.sleep()
