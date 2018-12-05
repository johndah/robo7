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
import random

training_filename = '/home/jtao/robo7_vision/data/Classify/train.tfrecord'
validation_filename = '/home/jtao/robo7_vision/data/Classify/validation.tfrecord'

imgDir = '/home/jtao/robo7_vision/data/Classify/train/G_cube26.png'
ckpt_dir ='./checkpoints'
checkpoint = ckpt_dir + '/mobilenetv2-625'

dataDir = '/home/jtao/robo7_vision/data/'
imgDir = dataDir + 'Classify/train/*.png'


def load(sess, saver, checkpoint_dir):
    import re
    print("[*] Reading checkpoints...")

    ckpt = tf.train.get_checkpoint_state(checkpoint_dir)
    if ckpt and ckpt.model_checkpoint_path:
        ckpt_name = os.path.basename(ckpt.model_checkpoint_path)
        # ckpt_name = 'mobilenetv2-100'
        saver.restore(sess, os.path.join(checkpoint_dir, ckpt_name))
        counter = int(next(re.finditer("(\d+)(?!.*\d)", ckpt_name)).group(0))
        print("[*] Success to read {}".format(ckpt_name))
        return True, counter
    else:
        print("[*] Failed to find a checkpoint")
        return False, 0


def decode(serialized_example):

    keys_to_features = {
        'image': tf.FixedLenFeature([], tf.string),
        'label': tf.FixedLenFeature([], tf.int64),
    }

    features = tf.parse_single_example(serialized_example, keys_to_features)

    image = tf.decode_raw(features['image'], tf.uint8)
    image = tf.cast(image, tf.float32)
    image = tf.reshape(image, shape=[args.width, args.height, 3])
    # image.set_shape([3, 60, 60, 3])

    label = tf.cast(features['label'], tf.int32)

    return image, label


def input_fn(train, batch_size, num_epochs):

    if train:
        filename = training_filename
    else:
        filename = validation_filename

    dataset = tf.data.TFRecordDataset(filenames=filename)

    dataset = dataset.map(decode)

    # dataset = dataset.repeat(num_epochs)

    dataset = dataset.batch(batch_size)

    return dataset


def pred():

    batch_s = 100

    tf.reset_default_graph()
    sess = tf.Session()

    dataset = input_fn(train=False, batch_size=batch_s, num_epochs=1)
    iterator = dataset.make_one_shot_iterator()
    image_batch, label_batch = iterator.get_next()

    logits, endpoints = mobilenetv2(image_batch, 14, is_train=False)

    # evaluate model, for classification
    correct_pred = tf.equal(tf.argmax(endpoints, 1), tf.cast(label_batch, tf.int64))
    acc = tf.reduce_mean(tf.cast(correct_pred, tf.float32))

    # sess=tf.Session()
    saver=tf.train.Saver()
    print('[*] Try to load trained model...')
    could_load, step = load(sess, saver, args.checkpoint_dir)

    accum = 0
    objNum = np.zeros(14, dtype=float)
    correctNum = np.zeros((14, 14), dtype=float)
    for i in range(50):

        try:
            la, _res, _pred, _acc = sess.run([label_batch, endpoints, correct_pred, acc])
            # _res, _acc = sess.run([endpoints, acc])

            # check acc of each class
            max_pro = np.argmax(_res, axis=1)
            for j in range(la.size):
                objNum[la[j]] = objNum[la[j]] + 1
                if _pred[j] == True:
                    correctNum[la[j], la[j]] = correctNum[la[j], la[j]] + 1
                else:
                    correctNum[la[j], max_pro[j]] = correctNum[la[j], max_pro[j]] + 1

            print(_acc * batch_s)

            accum = accum + _acc * batch_s

        except tf.errors.OutOfRangeError:
            print("end of dataset")

    print(accum)

    print("number of each class:", objNum)
    print("number of correctly classified, row is ground truth, column is result:", correctNum)
    print("acc of each class:", np.divide(np.diag(correctNum), objNum))

if __name__ == '__main__':
    pred()


a = 1

