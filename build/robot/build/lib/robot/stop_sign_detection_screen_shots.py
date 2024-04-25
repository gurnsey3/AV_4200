import cv2
import numpy as np

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Twist # Twist is for linear+angular velocities
from cv_bridge import CvBridge # onvert between ROS and OpenCV Images

from collections import deque
#from PIL import ImageGrab

class Stop(Node):

	def __init__(self):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a name
		super().__init__('Vision')
		      
		# Create the subscriber. This subscriber will receive an Image
		# from the video_frames topic. The queue size is 10 messages.
		self.subscription = self.create_subscription(
			Image, 
			'/camera/color/image_raw', 
			self.listener_callback, 
			10)
		self.subscription # prevent unused variable warning
		    # Used to convert between ROS and OpenCV images
		self.br = CvBridge()

	# Load haar cascade xml file
	stop_sign_cascade = cv2.CascadeClassifier('stop_sign_classifier_2.xml')

	def listener_callback(self, data):
		### Take screen shots
		# define screen shot area 
		# box = (960, 0, 1920, 720)
		#box=(0, 0, 960, 720) # smaller screen
		#image = ImageGrab.grab(box) # bbox specifies a region (bbox= x, y, w, h). x, y --> origin. w, h --> width, height
		#image = cv2.imread('media/stop.png')
		image = self.br.imgmsg_to_cv2(data)
		# Convert to numpy array
		#image_np = np.array(image) 
		# Convert to BGR(openCV format) and then to gray scale
		#image_np = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		# Apply Gaussian filter
		gray_filered = cv2.GaussianBlur(gray, (5, 5), 0)
		cv2.imshow('screenshot',gray_filered)
		
	    # Detection
		stop_signs = stop_sign_cascade.detectMultiScale(gray_filered, scaleFactor=1.05, minNeighbors=5, minSize=(5, 5))
		
		print(len(stop_signs))
		# Draw rectangels
		for (x,y,w,h) in stop_signs:
			cv2.rectangle(image_np, (x, y), (x+w, y+h), (255, 255, 0), 2)

		cv2.namedWindow("screenshot", cv2.WINDOW_NORMAL)
		cv2.resizeWindow('screenshot', 640, 480)
		cv2.imshow('screenshot',image_np)
def main(args=None):

	# Initialize the rclpy library
	rclpy.init(args=args)

	# Create the node
	stop = Stop()

	# Spin the node so the callback function is called.
	rclpy.spin(stop)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	stop.destroy_node()

	# Shutdown the ROS client library for Python
	rclpy.shutdown()
  
if __name__ == '__main__':
	main()
