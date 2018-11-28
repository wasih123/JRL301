#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import Image

class controller:
  def __init__(self):
	self.bridge = CvBridge()
	self.command_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped)
	#self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
	self.image_sub = rospy.Subscriber("/camera/zed/rgb/image_rect_color", Image, self.callback)

  def callback(self, data):
	try:
	  image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	  print e
	  return

	height, width, channels = image.shape
	#print image.tolist()
	#l and u are the lower and upper ranges for yellow color of line respectively
	l = np.array([0, 240, 240], dtype = "uint8")
	u = np.array([10, 255, 255], dtype = "uint8")

	mask = cv2.inRange(image, l, u)
	#print mask.tolist()
	final_img = cv2.bitwise_and(image, image, mask = mask)
	m = cv2.moments(mask, False)
	#moments are taken to calculate centroid of the yellow region
	try:
	  x = m['m10']/m['m00']
	  y = m['m01']/m['m00']
	except ZeroDivisionError:
	  x = width/2
	  y = height/2
	#draw a blue circle now
	cv2.circle(final_img, (int(x), int(y)), 2, (255, 0, 0), 3)
	
	#show both the original image and the final_img(yellow line extracted and blue circle drawn in its centre)
	cv2.imshow("IMG", np.hstack([image, final_img]))
	#np.hstack just concatenates the two image arrays along the columns and hence in the end it is just one image in one image window
	cv2.waitKey(1)
	
	#calculate the error in localizing now
	error = x - width/2
	#print error
	throttle = 0.5    #this will be constant
	if(error > 0):          #only steering angle will change based on error
		#this means we need to steer right
		steer = error/80    #norm factor
	else:
		steer = error/80    #norm factor
	
	#now publish an ackermann drive msg. to the controller  
	msg = AckermannDriveStamped();
	msg.header.stamp = rospy.Time.now();
	msg.header.frame_id = "base_link";
	msg.drive.speed = throttle
	msg.drive.acceleration = 1
	msg.drive.jerk = 1
	msg.drive.steering_angle = -1.0*steer
	msg.drive.steering_angle_velocity = 1
	self.command_pub.publish(msg)

def main(args):
  rospy.init_node('line_node', anonymous=True)
  c = controller()
  try:
	rospy.spin()
  except KeyboardInterrupt:
	print("Closing down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
