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
from acfDetector.msg import detectedObj
from classify.msg import classifiedObj

# Parameter setting
ckpt_dir ='../checkpoints/160_1/'
width = 160
height = 160

class classifier:

    def __init__(self):

        # initialize vaiables
        self.objClass = ["Yellow Ball", "Yellow Cube",
                        "Green Cube", "Green Cylinder", "Green Hollow Cube",
                        "Orange Cross", "Orange Star",
                        "Red Cyliner", "Red Hollow Cube", "Red Ball",
                        "Blue Cube", "Blue Triangle",
                        "Purple Cross", "Purple Star"]

        # Network define
        tf.reset_default_graph()
        self.inputs = tf.placeholder(tf.float32, [None, width, height, 3], name='inputs')

        self.logits, self.pred = mobilenetv2(self.inputs, 14, is_train=False)

        self.saver=tf.train.Saver()

        self.sess = tf.Session()

        ckpt = tf.train.get_checkpoint_state(ckpt_dir)
        if ckpt and ckpt.model_checkpoint_path:
            ckpt_name = os.path.basename(ckpt.model_checkpoint_path)
            self.saver.restore(self.sess, os.path.join(ckpt_dir, ckpt_name))
            print("[*] Success to read {}".format(ckpt_name))
        else:
            sys.exit("[*] Failed to find a checkpoint")

        self.bridge = CvBridge()

        self.detectedObj_sub = rospy.Subscriber("/vision/object", detectedObj, self.applyModel)
        self.result_pub = rospy.Publisher("/vision/result", classifiedObj, queue_size=1)
        
        # self.image_sub = rospy.Subscriber("/vision/object/img", Image, self.applyModel)
        # self.obj_class_pub = rospy.Publisher("/vision/object/class", Int16, queue_size=1)

    # def applyModel(self, data):
    #     origImg = self.bridge.imgmsg_to_cv2(data, "bgr8")

    #     origImg = cv2.resize(origImg, (width, height))

    #     img = origImg.astype(np.float32)

    #     feed_dict = {self.inputs: [img]}
    #     im, res = self.sess.run([self.inputs, self.pred], feed_dict=feed_dict)

    #     result_class = res.argmax()

    #     # put resulting text on image
    #     cv2.putText(origImg, self.objClass[result_class], (30, 20),
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    #     cv2.imshow("object image", origImg)
    #     cv2.waitKey(1000)

    #     print("pro:", res)
    #     rospy.loginfo("predict class: %d", result_class)

    #     self.obj_class_pub.publish(result_class)


    def applyModel(self, data):
        origImg = self.bridge.imgmsg_to_cv2(data.img, "bgr8")

        origImg = cv2.resize(origImg, (width, height))

        img = origImg.astype(np.float32)

        feed_dict = {self.inputs: [img]}
        im, res = self.sess.run([self.inputs, self.pred], feed_dict=feed_dict)

        result_class = res.argmax()

        # put resulting text on image
        cv2.putText(origImg, self.objClass[result_class], (30, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow("object image", origImg)
        cv2.waitKey(2)

        # print("pro:", res)
        # rospy.loginfo("predict class: %d", result_class)

        # publish result
        msg = classifiedObj()
        msg.objClass = result_class
        msg.pos = data.pos
        self.result_pub.publish(msg)


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
