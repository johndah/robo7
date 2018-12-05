import cv2
import glob
import os

dataNam = 'light/'
imgDir = '/home/jtao/robo7_vision/data/Train_clear/image/' + dataNam + '*.png'
annDir = '/home/jtao/robo7_vision/data/Train_clear/annotation/' + dataNam + '*.txt'

unclearDir = '/home/jtao/robo7_vision/data/Train_unclear/' + dataNam + '*.png'

undirs = glob.glob(unclearDir)
imgdirs = glob.glob(imgDir)
anndirs = glob.glob(annDir)

for dir_1 in undirs:
    name = dir_1.replace('/home/jtao/robo7_vision/data/Train_unclear/'+dataNam, '')
    name = name.replace('.png', '')
    for dir_img in imgdirs:
        if '/' + name + '.png' in dir_img:
            os.remove(dir_img)
    for dir_txt in anndirs:
        if '/' + name + '.txt' in dir_txt:
            os.remove(dir_txt)


