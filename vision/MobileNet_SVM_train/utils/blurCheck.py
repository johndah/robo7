import cv2
import glob

dir = '/home/jtao/robo7_vision/data/Train/image/light/*.png'
OutputDir = '/home/jtao/robo7_vision/data/Train_unclear/light/'
addrs = glob.glob(dir)

threshold = 12

for addr in addrs:
    img = cv2.imread(addr)
    # cv2.imshow("image", frame)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    score = cv2.Laplacian(gray, cv2.CV_64F).var()

    print(score)

    if score < threshold:
        cv2.imshow("image", img)
        imgName = addr.replace('/home/jtao/robo7_vision/data/Train/image/light/', '')

        cv2.imwrite(OutputDir+imgName, img)

        # while(1):
        #     key = cv2.waitKey(10)
        #     if key == 27:
        #         break
        #     if key == 32:
        #         break
        #
        # if key == 27:
        #     break


cv2.destroyAllWindows()
