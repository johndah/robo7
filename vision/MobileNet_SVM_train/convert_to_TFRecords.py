import sys
import glob
import cv2
import numpy as np
import tensorflow as tf

from random import shuffle

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

# to shuffle data
c = list(zip(addrs, labels))
shuffle(c)
addrs, labels = zip(*c)

# divide the data into 90% train, 10% validation
train_addrs = addrs[0:int(0.9*len(addrs))]
train_labels = labels[0:int(0.9*len(labels))]

val_addrs = addrs[int(0.9*len(addrs)):]
val_labels = labels[int(0.9*len(labels)):]


def load_image(addr):
  # read an image and resize to (224, 224)
  # cv2 load images as BGR, convert it to RGB
  img = cv2.imread(addr)
  # img = cv2.resize(img, (224, 224), interpolation=cv2.INTER_CUBIC)
  # cv2.imshow('Original', img)
  #
  #
  # k = cv2.waitKey(5) & 0xFF
  #
  #
  # img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
  # cv2.imshow('Converted', img)
  #
  # k = cv2.waitKey(5) & 0xFF

  # img = img.astype(np.float32)
  return img


def _int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))


def _bytes_feature(value):
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))


def createDataRecord(out_filename, addrs, labels):
  # open the TFRecords file
  writer = tf.python_io.TFRecordWriter(out_filename)
  for i in range(len(addrs)):
    # print how many images are saved every 1000 images
    if not i % 1000:
      print('Train data: {}/{}'.format(i, len(addrs)))
      sys.stdout.flush()
    # Load the image
    img = load_image(addrs[i])

    label = labels[i]

    if img is None:
      continue

    # Create a feature
    feature = {
      'image': _bytes_feature(img.tostring()),
      'label': _int64_feature(label)
    }
    # Create an example protocol buffer
    example = tf.train.Example(features=tf.train.Features(feature=feature))

    # Serialize to string and write on the file
    writer.write(example.SerializeToString())

  writer.close()
  sys.stdout.flush()


def main():
  train_filename = dataDir + 'Classify/train'  # address to save the TFRecords file
  val_filename = dataDir + 'Classify/validation'

  # max_num = 3000
  # for i in range(len(train_addrs)//max_num + 1):
  #   end = min((i+1)*max_num-1, len(train_addrs)-1)
  #   createDataRecord(train_filename+str(i)+'.tfrecord', train_addrs[i*max_num:end], train_labels[i*max_num:end])
  #
  # for i in range(len(val_addrs)//max_num + 1):
  #   end = min((i+1)*max_num-1, len(val_addrs)-1)
  #   createDataRecord(val_filename+str(i)+'.tfrecord', val_addrs[i*max_num:end], val_labels[i*max_num:end])

  createDataRecord(train_filename+'.tfrecord', train_addrs, train_labels)
  createDataRecord(val_filename+'.tfrecord', val_addrs, val_labels)

if __name__ == '__main__':
  main()
