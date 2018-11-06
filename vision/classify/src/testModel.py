#!/usr/bin/env python
from __future__ import print_function

import rospy
import os
import tensorflow as tf
import cv2
import numpy as np
import sys

from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from mobilenet_v2 import mobilenetv2

ckpt_dir ='../checkpoints'
width = 60
height = 60


class classifier:

    def __init__(self):

        # Network define
        tf.reset_default_graph()
        self.inputs = tf.placeholder(tf.float32, [None, width, height, 3], name='inputs')

        self.logits, self.pred = mobilenetv2(self.inputs, 14, is_train=False)

        self.saver=tf.train.Saver()

        self.sess = tf.Session()
        # saver.restore(sess, checkpoint)

        ckpt = tf.train.get_checkpoint_state(ckpt_dir)
        if ckpt and ckpt.model_checkpoint_path:
            ckpt_name = os.path.basename(ckpt.model_checkpoint_path)
            self.saver.restore(self.sess, os.path.join(ckpt_dir, ckpt_name))
            print("[*] Success to read {}".format(ckpt_name))
        else:
            sys.exit("[*] Failed to find a checkpoint")


        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/vision/object/img", Image, self.applyModel)
        self.obj_class_pub = rospy.Publisher("/vision/object/class", Int16, queue_size=1)

    ##      for testing         ##
    #     self.obj_true_class_sub = rospy.Subscriber("/vision/object/trueClass", Int16, self.callback)
    #     self.trueClass = 100
    #     self.numImg = 0
    #     self.trueImg = 0


    # def callback(self, data):
    #     self.trueClass = data.data

    ##      for testing      ##

    def applyModel(self, data):
        origImg = self.bridge.imgmsg_to_cv2(data, "bgr8")

        origImg = cv2.resize(origImg, (60, 60))

        cv2.imshow("object image", origImg)
        cv2.waitKey(3)

        img = origImg.astype(np.float32)

        # with tf.Session as sess:

        feed_dict = {self.inputs: [img]}
        im, res = self.sess.run([self.inputs, self.pred], feed_dict=feed_dict)

        result_class = res.argmax()

        print("pro:", res)
        rospy.loginfo("predict class: %d", result_class)


        ##      for testing         ##
        # self.numImg = self.numImg + 1
        # rospy.loginfo("true class: %d", self.trueClass)
        # if self.trueClass == result_class:
        #     rospy.loginfo("True !!")
        #     self.trueImg = self.trueImg + 1
        # else:
        #     rospy.loginfo("False !!")

        # rospy.loginfo("acc right now: %f", float(self.trueImg) / self.numImg)
        ##      for testing         ##


        self.obj_class_pub.publish(result_class)


def main():

    ic = classifier()
    rospy.init_node('testModel', anonymous=True)
    rate = rospy.Rate(5)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down the classifier")

    cv2.destoryAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass