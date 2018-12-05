import cv2

imgDir = '/home/jtao/robo7_vision/data/Classify/collect/G_cylinder/G_cylinder_a160.png'

img = cv2. imread(imgDir)

cv2.imshow('img', img)
cv2.waitKey(10)

edges = cv2.Canny(img, 10, 50)

cv2.imshow('edge', edges)
cv2.waitKey(10)

a = 1