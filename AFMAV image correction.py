"""
Created on Wed Mar  3 12:01:37 2021

@author: Levi


Image calibration using Zhangs method
Z. Zhang. A flexible new technique for camera calibration. 
IEEE Transactions on Pattern Analysis and Machine Intelligence (T-PAMI), 
22(11):1330 -1334, 2000.
"""


#%matplotlib inline
#%matplotlib qt

# im = cv2.imread('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/87214770.jpg')
# plt.imshow(im)


import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#9,6 8,8 bigger
#6,9 8,8 smaller
nhor = 9
nver = 6
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((nver*nhor,3), np.float32)
objp[:,:2] = np.mgrid[0:nver,0:nhor].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    # img = cv2.rotate(img,cv2.cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (nver,nhor),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(8,8),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (nver,nhor), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

img = cv2.imread('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/33148513.jpg')
h,  w = img.shape[:2]
alpha = 1
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),alpha,(w,h))


# undistort
# dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# # crop the image
# x,y,w,h = roi
# dst = dst[y:y+h, x:x+w]
# cv2.imwrite('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/calibresult.png',dst)
# cv2.imshow('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/calibresult.png',dst)

# undistort
mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/calibresult.png',dst)
cv2.imshow('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/calibresult.png',dst)

for filename in images:
    img = cv2.imread(filename)
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),alpha,(w,h))

    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

    x,y,w,h = roi
    # dst = dst[ y:, x:]

    cv2.imshow('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/calibresult.png',dst)
    cv2.waitKey(500)
    
# cv2.destroyAllWindows()