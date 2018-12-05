import tensorflow as tf
import numpy as np
from nets.mobilenet import mobilenet_v2
from mobilenet_v2 import mobilenetv2
from params import *

import time
import os
import cv2
import glob

training_filename = '/home/jtao/robo7_vision/data/Classify/train.tfrecord'
validation_filename = '/home/jtao/robo7_vision/data/Classify/validation.tfrecord'


def load(sess, saver, checkpoint_dir):
    import re
    print("[*] Reading checkpoints...")

    ckpt = tf.train.get_checkpoint_state(checkpoint_dir)
    if ckpt and ckpt.model_checkpoint_path:
        ckpt_name = os.path.basename(ckpt.model_checkpoint_path)
        saver.restore(sess, os.path.join(checkpoint_dir, ckpt_name))
        counter = int(next(re.finditer("(\d+)(?!.*\d)", ckpt_name)).group(0))
        print("[*] Success to read {}".format(ckpt_name))
        return True, counter
    else:
        print("[*] Failed to find a checkpoint")
        return False, 0


def decode_train(serialized_example):

    keys_to_features = {
        'image': tf.FixedLenFeature([], tf.string),
        'label': tf.FixedLenFeature([], tf.int64),
    }

    features = tf.parse_single_example(serialized_example, keys_to_features)

    image = tf.decode_raw(features['image'], tf.uint8)

    image = tf.cast(image, tf.float32)
    image = tf.reshape(image, shape=[args.width, args.height, 3])

    image = tf.image.random_brightness(image, 0.1)
    image = tf.image.random_flip_left_right(image)

    label = tf.cast(features['label'], tf.int32)

    return image, label


def decode_valid(serialized_example):

    keys_to_features = {
        'image': tf.FixedLenFeature([], tf.string),
        'label': tf.FixedLenFeature([], tf.int64),
    }

    features = tf.parse_single_example(serialized_example, keys_to_features)

    image = tf.decode_raw(features['image'], tf.uint8)
    image = tf.cast(image, tf.float32)
    image = tf.reshape(image, shape=[args.width, args.height, 3])

    label = tf.cast(features['label'], tf.int32)
    return image, label


def input_fn(train, batch_size, num_epochs):

    if train:
        filename = training_filename
        dataset = tf.data.TFRecordDataset(filenames=filename)
        dataset = dataset.map(decode_train)
    else:
        filename = validation_filename
        dataset = tf.data.TFRecordDataset(filenames=filename)
        dataset = dataset.map(decode_valid)

    dataset = dataset.shuffle(buffer_size=10000)

    dataset = dataset.repeat(num_epochs)

    dataset = dataset.batch(batch_size)

    iterator = dataset.make_one_shot_iterator()

    return iterator


def run_training():

    tf.reset_default_graph()

    sess = tf.Session()

    # image_batch, label_batch = input_fn(
    #     train=True, batch_size=args.batch_size, num_epochs=args.epoch)

    iterator_train = input_fn(train=True, batch_size=args.batch_size, num_epochs=args.epoch)
    iterator_valid = input_fn(train=False, batch_size=500, num_epochs=args.epoch)


    isTrain = tf.placeholder(tf.bool, shape=())
    image_batch, label_batch = tf.cond(tf.equal(isTrain, tf.constant(True)), lambda: iterator_train.get_next(), lambda: iterator_valid.get_next())

    # with tf.contrib.slim.arg_scope(mobilenet_v2.training_scope(is_training=True)):
    #     logits, endpoints = mobilenet_v2.mobilenet(image_batch, num_classes=14)
    logits, pred = mobilenetv2(image_batch, num_classes=args.num_classes, is_train=isTrain)

    # loss
    loss = tf.reduce_mean(
        tf.nn.sparse_softmax_cross_entropy_with_logits(labels=label_batch, logits=logits))

    # L2 regularization
    l2_loss = tf.add_n(tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES))
    total_loss = loss + l2_loss

    # learning rate decay
    base_lr = tf.constant(args.learning_rate)
    lr_decay_step = args.num_samples // args.batch_size * 2  # every epoch
    global_step = tf.placeholder(dtype=tf.float32, shape=())
    lr = tf.train.exponential_decay(base_lr, global_step=global_step, decay_steps=lr_decay_step,
                                    decay_rate=args.lr_decay)

    # optimizer
    update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
    with tf.control_dependencies(update_ops):
    #     tf.train.RMSPropOptimizer(learning_rate=self.lr, decay=0.9, momentum=0.9)
        train_op = tf.train.AdamOptimizer(
            learning_rate=lr, beta1=args.beta1).minimize(total_loss)

    # evaluate model, for classification with training data
    correct_pred = tf.equal(tf.argmax(pred, 1), tf.cast(label_batch, tf.int64))
    acc = tf.reduce_mean(tf.cast(correct_pred, tf.float32))

    # summary
    tf.summary.scalar('total_loss', total_loss)
    tf.summary.scalar('accuracy', acc)
    tf.summary.scalar('learning_rate', lr)
    summary_op = tf.summary.merge_all()

    # summary writer
    writer = tf.summary.FileWriter(args.logs_dir, sess.graph)

    sess.run(tf.global_variables_initializer())

    # saver for save/restore model
    saver = tf.train.Saver()
    # load pretrained model

    if not args.renew:
        print('[*] Try to load trained model...')
        could_load, step = load(sess, saver, args.checkpoint_dir)
    step = 0

    max_steps = int(args.num_samples / args.batch_size * args.epoch)
    print('max step:', max_steps)

    print('START TRAINING...')
    _acc_valid = 0
    con_step = 0
    for _step in range(step + 1, max_steps + 1):
        start_time = time.time()
        # feed_dict = {global_step: _step, inputs: image_batch, truth: label_batch}
        # feed_dict = {global_step: _step}
        feed_dict = {global_step: _step, isTrain: True}


        # _, _lr = sess.run([train_op, lr], feed_dict=feed_dict)
        _, _, _, _lr, _res, _acc = sess.run([image_batch, label_batch, train_op, lr, pred, acc], feed_dict=feed_dict)

        # sample_img = im[0].astype(np.uint8)
        # cv2.imshow('image', sample_img)
        # cv2.waitKey(5)
        # print(la[0])

        # a = _res['Predictions']
        # b = a.argmax()
        # print logs and write summary
        if _step % 10 == 0:
            feed_dict = {global_step: _step, isTrain: True}
            _summ, _loss, _acc_valid, _pred = sess.run([summary_op, total_loss, acc, correct_pred],
                                          feed_dict=feed_dict)
            writer.add_summary(_summ, _step)
            print('global_step:{0}, time:{1:.3f}, lr:{2:.8f}, acc_valid:{3:.6f}, loss:{4:.6f}'.format
                  (_step, time.time() - start_time, _lr, _acc_valid, _loss))

        # save model
        if _step % 50 == 0:
            save_path = saver.save(sess, os.path.join(args.checkpoint_dir, args.model_name), global_step=_step)
            print('Current model saved in ' + save_path)

        print("acc:", _acc)
        if _acc_valid > 0.90:
            con_step = con_step + 1
            if con_step >= 3:
                break
        else:
            con_step = 0
        _acc_valid = 0

    tf.train.write_graph(sess.graph_def, args.checkpoint_dir, args.model_name + '.pb')
    save_path = saver.save(sess, os.path.join(args.checkpoint_dir, args.model_name), global_step=max_steps)

    # save_path = saver.save(sess, "checkpoints/model.ckpt")
    print('Final model saved in ' + save_path)
    sess.close()
    print('FINISHED TRAINING.')


if __name__ == '__main__':
    run_training()