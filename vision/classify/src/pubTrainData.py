#!/usr/bin/env python
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import glob
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


dataDir = '/home/jtao/robo7_vision/data/'
train_imgDir = dataDir + 'Classify/train/*.png'

addrs = glob.glob(train_imgDir)

labels = []
for addr in addrs:
	if 'Y_ball' in addr:
		labels.append(0)
	elif 'Y_cube' in addr:
		labels.append(1)
	elif 'G_cube' in addr:
		labels.append(2)
	elif 'G_cylinder' in addr:
		labels.append(3)
	elif 'G_hollow' in addr:
		labels.append(4)
	elif 'O_cross' in addr:
		labels.append(5)
	elif 'O_star' in addr:
		labels.append(6)
	elif 'R_cylinder' in addr:
		labels.append(7)
	elif 'R_hollow' in addr:
		labels.append(8)
	elif 'R_ball' in addr:
		labels.append(9)
	elif 'B_cube' in addr:
		labels.append(10)
	elif 'B_triangle' in addr:
		labels.append(11)
	elif 'P_cross' in addr:
		labels.append(12)
	elif 'P_star' in addr:
		labels.append(13)
	else:
		sys.exit("Unexpected image name!")


class image_converter:

  def __init__(self):
	self.image_pub = rospy.Publisher("/vision/object/img",Image)
	self.obj_class_pub = rospy.Publisher("/vision/object/trueClass", Int16, queue_size=1)


	self.bridge = CvBridge()
	# self.image_sub = rospy.Subscriber("/vision/object/class",Int16,self.callback)

  def pubData(self, image, label):

	cv2.imshow("Image window", image)
	cv2.waitKey(3)

	self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
	# except CvBridgeError as e:
	#   print(e)

	self.obj_class_pub.publish(label)


def main():
  ic = image_converter()
  rospy.init_node('pubTrainData', anonymous=True)

  rate = rospy.Rate(2)
  i = 1
  while not rospy.is_shutdown():
	image = cv2.imread(addrs[i])
	label = labels[i]
	ic.pubData(image, label)
	i = i + 1
	if i == 4000:
		i = 1
	rate.sleep()
  # try:
  #   rospy.spin()
  # except KeyboardInterrupt:
  #   print("Shutting down")
  # cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
