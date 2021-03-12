import numpy as np
import cv2
import glob


print('showing undistorted images')

images = glob.glob('../paparazzi/calibration_frontcam/20190121-163447/*.jpg')
images.sort()
calib=np.load('calibration_result.npz')

print(calib)
mtx = calib['mtx']
dist = calib['dist']
print(mtx,dist)

for filename in images:
    img = cv2.imread(filename)

    dst = cv2.undistort(img,mtx,dist,None,mtx)

    cv2.imshow('img',dst)
    cv2.waitKey(50)
    
#cv2.imwrite('../paparazzi/calibration_frontcam/20190121-163447/test/calibresult.png',dst)
#cv2.imshow('../paparazzi/calibration_frontcam/20190121-163447/test/calibresult.png',dst)

print('done')
cv2.destroyAllWindows()
