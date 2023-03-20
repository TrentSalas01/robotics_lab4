#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True

	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('flip_image', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			# set top left corner coords
			rect1 = (0, 0)
			# set bottom right corner coords
			rect2 = (1080, 720)
			# assign variable to color white
			color = (0, 0, 0)
			# flip the image up
			hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
			# set lower bound
			lower_yellow_hsv = np.array([25,1,1])
			# set upper bound
			upper_yellow_hsv = np.array([60,255,255])
			# define the upper and lower ranges
			yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
			# set up the rectangle to shade corners black
			yellow_mask = cv2.rectangle(yellow_mask, rect1, rect2, color, 350)
			# convert it to ros msg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(yellow_mask, encoding="mono8")
			# publish the image
			img_pub.publish(img_msg)
		# pause until the next iteration			
		rate.sleep()



