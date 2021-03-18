#include "Calibrate.h"
//#include <utility> //For the std::pair return, not needed if changing private variables instead
#include <opencv2>

//#include camerainput

Calibrate::Calibrate(int image)
{
    Calibratecam();
    Undistort(image);
}

void Calibrate::Calibratecam()
{
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001);
    nhor = 9;
    nver = 6;

    //Some loop (Time, amount of images, amount of ret==True images etc)

    //Get cameraoutput / images over a longer time, not just one
    img = camerainput;
    gray = cv2::cvtColor(img,cv2.COLOR_BGR2GRAY);

    ret, corners = cv2::findChessboardCorners(gray, (nver,nhor),None);

    if ret == True:
        objpoints.append(objp);

        corners2 = cv2.cornerSubPix(gray,corners,(8,8),(-1,-1),criteria);
        imgpoints.append(corners2);

        //# Draw and display the corners
        //img = cv2.drawChessboardCorners(img, (nver,nhor), corners2,ret);

    //End loop

    ret, newMtx, newDist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None);
    
    mtx = newMtx;
    dist = newDist;
}

int Calibrate::Undistort(int image)
{
    dst = cv2::undistort(image, mtx, dist, None, mtx);
    img = cv2::imwrite(image, dst);

    /* //Still mostly python code, not C++
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2::getOptimalNewCameraMatrix(mtx,dist,(w,h),alpha,(w,h))
    mapx,mapy = cv2::initUndistortRectifyMap(mtx,dist,None,mtx,(w,h),5)
    dst = cv2::remap(img,mapx,mapy,cv2.INTER_LINEAR)

    x,y,w,h = roi
    dst = dst[ y:, x:]

    img = cv2.imwrite('C:/Users/Levi/Desktop/AE4317_2019_datasets/calibration_frontcam/20190121-163447/Test/calibresult.png',dst)
    */
    
    return img;
}
