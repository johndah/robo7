import tensorflow as tf
from nets.mobilenet import mobilenet_v2
from mobilenet_v2 import mobilenetv2
from params import *

from IPython import display

import os
import time
import numpy as np
import cv2
import glob
import sys
from random import shuffle

ckpt_dir ='./checkpoints'
# checkpoint = ckpt_dir + '/mobilenetv2-625'

folderName = ['Y_ball', 'Y_cube', 'G_cube', 'G_cylinder', 'G_hollow', 'O_cross', 'O_star',
              'R_cylinder', 'R_hollow', 'R_ball', 'B_cube', 'B_triangle', 'P_cross', 'P_star']

dataDir = '/home/jtao/robo7_vision/data/'
dataDir1 = '/home/jtao/robo7_vision/data/Classify/train/'
dataDir2 = '/home/jtao/robo7_vision/data/Classify/collect/'

addrs = []
for i in range(len(folderName)):
  imgDir = dataDir1 + folderName[i] + '/*.png'
  addrs = addrs + glob.glob(imgDir)
  imgDir = dataDir2 + folderName[i] + '/*.png'
  addrs = addrs + glob.glob(imgDir)

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


c = list(zip(addrs, labels))
shuffle(c)
addrs, labels = zip(*c)

# model_file

tf.reset_default_graph()

images = tf.placeholder(tf.float32, [None, args.width, args.height, 3], name='inputs')

logits, pred = mobilenetv2(images, 14, is_train=False)

sess = tf.Session()
saver=tf.train.Saver()

ckpt = tf.train.get_checkpoint_state(ckpt_dir)
if ckpt and ckpt.model_checkpoint_path:
    ckpt_name = os.path.basename(ckpt.model_checkpoint_path)
    # ckpt_name = 'mobilenetv2-625'
    saver.restore(sess, os.path.join(ckpt_dir, ckpt_name))
    print("[*] Success to read {}".format(ckpt_name))
else:
    print("[*] Failed to find a checkpoint")

acc = 0

objNum = np.zeros(14, dtype=float)
correctNum = np.zeros(14, dtype=float)
# accMatrix = np.zeros(14, dtype=float)

for i in range(len(addrs)):

    # index = random.randint(1, 3000)
    index = i

    img = cv2.imread(addrs[index])

    cv2.imshow('image', img)
    cv2.waitKey(2)

    img = img.astype(np.float32)

    feed_dict = {images: [img]}
    # im, res = sess.run([images, pred], feed_dict=feed_dict)
    # im = im[0]

    res = sess.run([pred], feed_dict=feed_dict)
    res = res[0]

    print("ground true:", labels[index])
    print("x prob:", res)
    print("Top 1 prediction: ", res.argmax(), res.max())
    print("--------------------------------------------")
    print(round(res.max(), 2))

    objNum[labels[index]] = objNum[labels[index]] + 1
    if np.equal(res.argmax(), labels[index]):

        acc = acc + 1
        print(acc)
        correctNum[labels[index]] = correctNum[labels[index]] + 1
    # else:
    #     cv2.imshow('image', img)
    #     while(1):
    #         key = cv2.waitKey(10)
    #         if key == 32:
    #             break

print(acc)
print("number of each class:", objNum)
print("number of correctly classified:", correctNum)
print("acc of each class:", np.divide(correctNum, objNum))

a = 1
