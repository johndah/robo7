dir = '/home/jtao/robo7_vision/ACF_detection/ras/models/Luv+Mag_3data/test.avi';
v = VideoReader(dir);

while hasFrame(v)
    image = readFrame(v);
    imshow(image);
    pause;
end