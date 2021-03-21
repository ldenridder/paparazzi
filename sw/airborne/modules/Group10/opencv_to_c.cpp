
#include "std.h"
#include "opencv_to_c.h"

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
//#include <iostream> //for printing to terminal
using namespace cv;



struct image_t *imageProcess(struct image_t *image)
{
  WIDTH = 520;
  HEIGHT = 240;
  Mat M(height, width, CV_8UC2, img);
  //Mat imgOLD;
  //imgOLD = imread("../C-code/30315196.jpg"); //get image from file (need to change)
  Mat image;
  Mat mtx;
  Mat dist;

  cvtColor(M, image, CV_YUV2BGR_UYVY);//CV_8UC2 OR CV_YUV2BGR_Y422); //convert to MAT
  //mtx = getMtx(mtx);
  //dist = getDist(dist);
  
  //Camera parameters now done by hand, not by the program reading a file created with calibration
  float fx, cx, fy, cy;
  fx = 313.59567215;
  cx = 34.88945335;
  fy = 315.80797773;
  cy = 258.08732166;
  mtx = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  float k1,k2,k3,k4,k5,k6,p1,p2;
  k1 = -0.33432092;
  k2 = 0.13049227;
  k3 = -0.02783748;
  p1 = 0.00087944;
  p2 = 0.00059761;
  dist = (Mat1d(1, 5) << k1, k2, p1, p2, k3);

  //undistort(imgOLD, M, mtx, dist, mtx);
  undistort(M, M, mtx, dist, mtx);
  //imwrite("../C-code/__TEST.jpg", M); //change image
  //std::cout << "M = " << std::endl << " "  << mtx << std::endl << std::endl;
	
	cvtColor(M, image, CV_YUV2GRAY_Y422);
  return image;
}

void maximumBoxFilter(int n, int *p_hmat_z, int *p_imgResult){
	size = (n,n);
	shape = cv2.MORPH_RECT;
	kernel = cv2.getStructuringElement(shape,size);
	
	imgResult = cv2.dilate(hmat_z,kernal); //Not sure if arrays will work here
}

void noiseFilter(int *p_hmat_z,int X,int Y){
	int x,y;
	
	maximumBoxFilter(3,p_hmat_z,p_hmat_z);
	fch = 50;
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			if(p_hmat_z[y*X+x] < fch){
				p_hmat_z[y*X+x] = 0;
			}
			else{
				p_hmat_z[y*X+x] = 1;
			}
		}
	}
}


/*
void convertcolor()
{
	
}
*/


/*
void getImage()
{
  Mat image = imread("/home/levi/calibration_frontcam/30315196.jpg");
}

int getMtx(Mat mtx) //LOAD MTX AND DIST FROM A STORED FILE
{
  float fx, cx, fy, cy;
  fx = 313.59567215;
  cx = 34.88945335;
  fy = 315.80797773;
  cy = 258.08732166;
  mtx = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

}

int getDist(Mat dist)
{

  float k1,k2,k3,k4,k5,k6,p1,p2;
  k1 = -0.33432092;
  k2 = 0.13049227;
  k3 = -0.02783748;
  p1 = 0.00087944;
  p2 = 0.00059761;
  //Mat distortionCoefficients = (Mat1d(1, 4) << k1, k2, p1, p2);
  dist = (Mat1d(1, 5) << k1, k2, p1, p2, k3);
  //Mat distortionCoefficients = (Mat1d(1, 8) << k1, k2, p1, p2, k3, k4, k5, k6);
  
}
*/
