#!/usr/bin/env python3

################################################################################
## {Description}: Accessing raspicam/usbcam
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
from __future__ import division
import sys
import cv2
import time
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class CameraPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False
		
		self.alpha = 0.5

		self.pump0 = Bool()
		self.pump1 = Bool()
		self.pump2 = Bool()
		self.pump3 = Bool()
		self.fan = Bool()
		self.light = Bool()
		
		self.light.data = False
		self.lightStatus = False
		
		self.fan.data = False
		self.fanStatus = False
		
		rospy.logwarn("CameraPreview Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to Image msg
		self.image_topic = "/cv_camera_nuc/image_raw"
#		self.image_topic = "/camera/rgb/image_raw"
		self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cbImage)

		# Subscribe to CameraInfo msg
		self.cameraInfo_topic = "/cv_camera_nuc/camera_info"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(self.cameraInfo_topic, CameraInfo, 
			self.cbCameraInfo)

		# Subscribe to Int64 msg
		self.fingerTipX_topic = "/fingerTipX"
		self.fingerTipX_sub = rospy.Subscriber(self.fingerTipX_topic, Int64, 
			self.cbFingerTipX)

		# Subscribe to Int64 msg
		self.fingerTipY_topic = "/fingerTipY"
		self.fingerTipY_sub = rospy.Subscriber(self.fingerTipY_topic, Int64, 
			self.cbFingerTipY)
			
		# Subscribe to Int64 msg
		self.valSoil0_topic = "/val_soil0"
		self.valSoil0_sub = rospy.Subscriber(self.valSoil0_topic, Int64, 
			self.cbSoil0)

		# Subscribe to Int64 msg
		self.valSoil1_topic = "/val_soil1"
		self.valSoil1_sub = rospy.Subscriber(self.valSoil1_topic, Int64, 
			self.cbSoil1)

		# Subscribe to Int64 msg
		self.valSoil2_topic = "/val_soil2"
		self.valSoil2_sub = rospy.Subscriber(self.valSoil2_topic, Int64, 
			self.cbSoil2)

		# Subscribe to Int64 msg
		self.valSoil3_topic = "/val_soil3"
		self.valSoil3_sub = rospy.Subscriber(self.valSoil3_topic, Int64, 
			self.cbSoil3)

		# Subscribe to Float64 msg
		self.valHumid_topic = "/val_Humid"
		self.valHumid_sub = rospy.Subscriber(self.valHumid_topic, Float64, 
			self.cbHumid)

		# Subscribe to Float64 msg
		self.valTemp_topic = "/val_Temp"
		self.valTemp_sub = rospy.Subscriber(self.valTemp_topic, Float64, 
			self.cbTemp)

		# Subscribe to Bool msg
		self.valPump0_topic = "/pump0"
		self.valPump0_sub = rospy.Subscriber(self.valPump0_topic, Bool, 
			self.cbValPump0)

		# Subscribe to Bool msg
		self.valPump1_topic = "/pump1"
		self.valPump1_sub = rospy.Subscriber(self.valPump1_topic, Bool, 
			self.cbValPump1)

		# Subscribe to Bool msg
		self.valPump2_topic = "/pump2"
		self.valPump2_sub = rospy.Subscriber(self.valPump2_topic, Bool, 
			self.cbValPump2)

		# Subscribe to Bool msg
		self.valPump3_topic = "/pump3"
		self.valPump3_sub = rospy.Subscriber(self.valPump3_topic, Bool, 
			self.cbValPump3)

		# Publish to Bool msg
		self.pump0_topic = "/pump0"
		self.pump0_pub = rospy.Publisher(self.pump0_topic, Bool, 
			queue_size=10)

		# Publish to Bool msg
		self.pump1_topic = "/pump1"
		self.pump1_pub = rospy.Publisher(self.pump1_topic, Bool, 
			queue_size=10)

		# Publish to Bool msg
		self.pump2_topic = "/pump2"
		self.pump2_pub = rospy.Publisher(self.pump2_topic, Bool, 
			queue_size=10)

		# Publish to Bool msg
		self.pump3_topic = "/pump3"
		self.pump3_pub = rospy.Publisher(self.pump3_topic, Bool, 
			queue_size=10)

		# Publish to Bool msg
		self.light_topic = "/light"
		self.light_pub = rospy.Publisher(self.light_topic, Bool, 
			queue_size=10)

		# Publish to Bool msg
		self.fan_topic = "/fan"
		self.fan_pub = rospy.Publisher(self.fan_topic, Bool, 
			queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(2)

	def cbHumid(self, msg):
		self.valHumid = msg.data

	def cbTemp(self, msg):
		self.valTemp = msg.data

	def cbValPump0(self, msg):
		self.valPump0 = msg.data

	def cbValPump1(self, msg):
		self.valPump1 = msg.data

	def cbValPump2(self, msg):
		self.valPump2 = msg.data

	def cbValPump3(self, msg):
		self.valPump3 = msg.data

	def cbSoil0(self, msg):
		self.soil0 = msg.data

	def cbSoil1(self, msg):
		self.soil1 = msg.data

	def cbSoil2(self, msg):
		self.soil2 = msg.data

	def cbSoil3(self, msg):
		self.soil3 = msg.data

	def cbFingerTipX(self, msg):
		self.fingerTipX = msg.data

	def cbFingerTipY(self, msg):
		self.fingerTipY = msg.data
		
	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

			# comment if the image is mirrored
#			self.cv_image = cv2.flip(self.cv_image, 1)
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

	# Image information callback
	def cbInfo(self):

		self.fontFace = cv2.FONT_HERSHEY_DUPLEX
		self.fontScale = 0.5
		self.color = (255, 255, 255)
		self.color1 = (0, 255, 0)
		self.thickness = 1
		self.lineType = cv2.LINE_AA
		self.bottomLeftOrigin = False # if True (text upside down)

		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

#		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
#			self.fontFace, self.fontScale, self.color, self.thickness, self.lineType, 
#			self.bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
#			self.fontFace, self.fontScale, self.color, self.thickness, self.lineType, 
#			self.bottomLeftOrigin)
#		cv2.putText(self.cv_image, "(Temp: %.2f, Humidiity: %.2f)" % (self.imgWidth, self.imgHeight), 
#			(self.imgWidth-100, self.imgHeight-10), self.fontFace, self.fontScale, 
#			self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
		cv2.putText(self.cv_image, "(Temp: %.2f, Humidiity: %.2f)" % (self.valTemp, self.valHumid), 
			(10, 20), self.fontFace, self.fontScale, 
			self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
			
	# Show the output frame
	def cbShowImage(self):
#		self.cv_image_clone = imutils.resize(
#						self.cv_image.copy(),
#						width=320
#						)

#		cv2.imshow("CameraPreview", self.cv_image_clone)
		cv2.imshow("CameraPreview", self.cv_image)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
			self.cbInfo()
			self.cbDrawBorder()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("CameraPreview Node [OFFLINE]...")
		cv2.destroyAllWindows()
		
	# draw border
	def cbDrawBorder(self):
		overlay = self.cv_image.copy()
#		output = self.cv_image.copy()
		
#		cv2.line(overlay, (0, self.imgHeight // 2), (self.imgWidth, self.imgHeight // 2), (0, 0, 255), 3)
		
#		cv2.line(overlay, (self.imgWidth // 4, self.imgHeight // 2), (self.imgWidth, self.imgHeight // 2), (0, 0, 255), 3)
		cv2.putText(overlay, "Light", 
				(self.imgWidth//2 - 200, self.imgHeight//2 + 200), self.fontFace, self.fontScale, 
				self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
		if (self.fingerTipX <= self.imgWidth//2 - 200 + 20 and self.fingerTipX >= self.imgWidth//2 - 200 - 20) and \
			(self.fingerTipY <= self.imgHeight//2 + 200 + 20 and self.fingerTipY >= self.imgHeight//2 + 200 - 20):
			if self.light.data == False and self.lightStatus == False:
				self.light.data = True
			else:
				self.light.data = False
#			cv2.circle(overlay, (self.imgWidth//2 - 200, self.imgHeight//2 + 200), 30, (255, 0, 255), -1)
#			cv2.putText(overlay, "P6", 
#				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
#				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
#			cv2.putText(overlay, "Red Chillis", 
#				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
#				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass
#			self.light.data = False

		cv2.putText(overlay, "Fan", 
				(self.imgWidth//2 + 200, self.imgHeight//2 + 200), self.fontFace, self.fontScale, 
				self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
		if (self.fingerTipX <= self.imgWidth//2 + 200 + 20 and self.fingerTipX >= self.imgWidth//2 + 200 - 20) and \
			(self.fingerTipY <= self.imgHeight//2 + 200 + 20 and self.fingerTipY >= self.imgHeight//2 + 200 - 20):
			if self.fan.data == False and self.fanStatus == False:
				self.fan.data = True
			else:
				self.fan.data = False
#			cv2.circle(overlay, (self.imgWidth//2 - 200, self.imgHeight//2 + 200), 30, (255, 0, 255), -1)
#			cv2.putText(overlay, "P6", 
#				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
#				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
#			cv2.putText(overlay, "Red Chillis", 
#				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
#				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass
#		cv2.circle(overlay, (self.imgWidth//2 + 200, self.imgHeight//2 + 200), 30, (255, 0, 255), -1)

		if (self.fingerTipX <= 150 + 20 and self.fingerTipX >= 150 - 20) and \
			(self.fingerTipY <= 150 + 20 and self.fingerTipY >= 150 - 20):
			cv2.circle(overlay, (150, 150), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "P6", 
				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
			cv2.putText(overlay, "Red Chillis", 
				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass

		if (self.fingerTipX <= 150 + 20 and self.fingerTipX >= 150 - 20) and \
			(self.fingerTipY <= 300 + 20 and self.fingerTipY >= 300 - 20):
			cv2.circle(overlay, (150, 300), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "P5", 
				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
			cv2.putText(overlay, "Red Chillis", 
				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass

		if (self.fingerTipX <= 250 + 20 and self.fingerTipX >= 250 - 20) and \
			(self.fingerTipY <= 150 + 20 and self.fingerTipY >= 150 - 20):
			cv2.circle(overlay, (250, 150), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "P4", 
				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
			cv2.putText(overlay, "Red Chillis", 
				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass

		if (self.fingerTipX <= 250 + 20 and self.fingerTipX >= 250 - 20) and \
			(self.fingerTipY <= 300 + 20 and self.fingerTipY >= 300 - 20):
			cv2.circle(overlay, (250, 300), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "P4", 
				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
			cv2.putText(overlay, "Red Chillis", 
				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass

#		cv2.circle(overlay, (400, 150), 20, (0, 0, 255), -1)

		if (self.fingerTipX <= 350 + 20 and self.fingerTipX >= 350 - 20) and \
			(self.fingerTipY <= 300 + 20 and self.fingerTipY >= 300 - 20):
			cv2.circle(overlay, (350, 300), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "P3", 
				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
			cv2.putText(overlay, "Red Chillis", 
				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass

		if (self.fingerTipX <= 500 + 20 and self.fingerTipX >= 500 - 20) and \
			(self.fingerTipY <= 150 + 20 and self.fingerTipY >= 150 - 20):
			cv2.circle(overlay, (500, 150), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "P2", 
				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
			cv2.putText(overlay, "Red Chillis", 
				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass
		
		if (self.fingerTipX <= 500 + 20 and self.fingerTipX >= 500 - 20) and \
			(self.fingerTipY <= 300 + 20 and self.fingerTipY >= 300 - 20):
			cv2.circle(overlay, (500, 300), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "P1", 
				(self.fingerTipX, self.fingerTipY + 40), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
			cv2.putText(overlay, "Red Chillis", 
				(self.fingerTipX, self.fingerTipY + 60), self.fontFace, self.fontScale, 
				self.color1, self.thickness, self.lineType, self.bottomLeftOrigin)
		else:
			pass
#		
		if (self.fingerTipX <= 600 + 20 and self.fingerTipX >= 600 - 20) and \
			(self.fingerTipY <= 400 + 20 and self.fingerTipY >= 400 - 20):
			cv2.circle(overlay, (600, 400), 20, (0, 255, 0), -1) #3
			cv2.putText(overlay, "%s" % (self.valPump3), 
				(self.fingerTipX, self.fingerTipY), self.fontFace, self.fontScale, 
				self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
			self.pump3.data = True
		else:
			cv2.circle(overlay, (600, 400), 20, (0, 0, 255), -1) #3
			self.pump3.data = False
		
		if (self.fingerTipX <= 600 + 20 and self.fingerTipX >= 600 - 20) and \
			(self.fingerTipY <= 60 + 20 and self.fingerTipY >= 60 - 20):
			cv2.circle(overlay, (600, 60), 20, (0, 255, 0), -1) #2
			cv2.putText(overlay, "%s" % (self.valPump2), 
				(self.fingerTipX, self.fingerTipY), self.fontFace, self.fontScale, 
				self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
			self.pump2.data = True
		else:
			cv2.circle(overlay, (600, 60), 20, (0, 0, 255), -1) #2
			self.pump2.data = False

		if (self.fingerTipX <= 20 + 20 and self.fingerTipX >= 20 - 20) and \
			(self.fingerTipY <= 340 + 20 and self.fingerTipY >= 340 - 20):
			cv2.circle(overlay, (20, 340), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "%s" % (self.valPump0), 
				(self.fingerTipX, self.fingerTipY), self.fontFace, self.fontScale, 
				self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
			self.pump0.data = True
		else:
			cv2.circle(overlay, (20, 340), 20, (0, 0, 255), -1)
			self.pump0.data = False

		if (self.fingerTipX <= 20 + 20 and self.fingerTipX >= 20 - 20) and \
			(self.fingerTipY <= 80 + 20 and self.fingerTipY >= 80 - 20):
			cv2.circle(overlay, (20, 80), 20, (0, 255, 0), -1)
			cv2.putText(overlay, "%s" % (self.valPump1), 
				(self.fingerTipX, self.fingerTipY), self.fontFace, self.fontScale, 
				self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
			self.pump1.data = True
		else:
			cv2.circle(overlay, (20, 80), 20, (0, 0, 255), -1)
			self.pump1.data = False
			
#		cv2.circle(overlay, (self.imgWidth-20, 400), 20, (255, 0, 255), -1)
#		cv2.circle(overlay, (self.imgWidth-20, 80), 20, (255, 0, 255), -1)
		
		cv2.circle(overlay, (self.fingerTipX, self.fingerTipY), 20, (0, 255, 0), -1)
#		cv2.putText(overlay, "%d : %d" % (self.fingerTipX, self.fingerTipY), 
#			(self.fingerTipX, self.fingerTipY), self.fontFace, self.fontScale, 
#			self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
		
		if (self.fingerTipX <= self.imgWidth//2 + 200 and self.fingerTipX >= self.imgWidth//2 - 200) and \
			(self.fingerTipY <= self.imgHeight//2 + 200 and self.fingerTipY >= self.imgHeight//2 - 200):
			cv2.putText(overlay, "Soils: %.2f" % ((self.soil0 + self.soil1 + self.soil2 + self.soil3) / 4), 
				(self.fingerTipX, self.fingerTipY), self.fontFace, self.fontScale, 
				self.color, self.thickness, self.lineType, self.bottomLeftOrigin)
		
		self.pump0_pub.publish(self.pump0.data)
		self.pump1_pub.publish(self.pump1.data)
		self.pump2_pub.publish(self.pump2.data)
		self.pump3_pub.publish(self.pump3.data)
		self.light_pub.publish(self.light.data)
		self.fan_pub.publish(self.fan.data)
		
		# apply the overlay
		cv2.addWeighted(overlay, self.alpha, self.cv_image, 1 - self.alpha, 0, self.cv_image)
#		print(self.imgHeight)

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_preview_macbook_nuc', anonymous=False)
	camera = CameraPreview()
	
	r = rospy.Rate(10)

	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
		r.sleep()
