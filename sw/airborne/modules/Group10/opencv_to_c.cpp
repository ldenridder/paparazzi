#include "opencv_to_c.h" //Modify this file
#include <stdio.h>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

void noiseFilter(int *p_hmat_z,int *hmat_z_new, int X,int Y){
	//printf("We got here");
	int x,y;

	//int Hmat_z[X*Y];
	maximumBoxFilter(10,p_hmat_z);

	int i,j;
	
	
	int vmat_z[X*Y];

	for(x=0;x<X*Y;x++){
		vmat_z[x] = *(p_hmat_z + x);
	}
	
	

	int fch = 50;
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			if(p_hmat_z[y*X+x] < fch){
				vmat_z[y*X+x] = 0;
			}
			else{
				vmat_z[y*X+x] = 1;
			}
		}
	}

	//int w;

	/*printf("vmat_z \n");
	for(w=0;w<100;w++){
		printf("%f ", vmat_z[w]);
	}*/
	
	

	for(x=0;x<X*Y;x++){
		p_hmat_z[x] = vmat_z[x];
		
	}
/*
	printf("v_mat after max filter ");
	for(i=0;i<Y;i++){
		for(j=0;j<X;j++){
			printf("%d ", vmat_z[i*X+j]);
		}
	printf("\n");
}
*/
	return; 
}


/*
	int fch = 50;
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			if(p_hmat_z[y*X+x] < fch){
				p_hmat_z[y*X+x] = (uint8_t)0;
			}
			else{
				p_hmat_z[y*X+x] = (uint8_t)1;
			}
		}
	}
	return;
}
*/

void maximumBoxFilter(int n, int *p_hmat_z){

	int i,j;

	int array[520][240];


	for(i=0;i<240;i++){
		for(j=0;j<520;j++){
			array[j][i] = p_hmat_z[i*520+j];
		}
	}
	
	/*
	Mat M(240,520,CV_8UC1,p_hmat_z);
	Mat imageResult;
	Mat element = getStructuringElement(MORPH_RECT, Size(n,n),Point(-1,-1));
	//morphologyEx(M, imageResult, MORPH_CLOSE, element);
	dilate(M, imageResult,element);
	*/
	Mat M(520,240,CV_8U,array);
	Mat imageResult;
	Mat element = getStructuringElement(MORPH_RECT, Size(2*n+1,2*n +1),Point(n,n));
	//morphologyEx(M, imageResult, MORPH_CLOSE, element);
	dilate(M, imageResult,element,Point(-1,-1),1);



	std::vector<int> result;

	if (imageResult.isContinuous()) {
  	// array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has problems for submatrix like mat = big_mat.row(i)
	  result.assign((int*)imageResult.data, (int*)imageResult.data + imageResult.total()*imageResult.channels());} 
	else {
  		for (int i = 0; i < imageResult.rows; ++i) {
    	result.insert(result.end(), imageResult.ptr<int>(i), imageResult.ptr<int>(i)+imageResult.cols*imageResult.channels());
  		}
	}

	p_hmat_z = result.data();


	return;
}




	//morphologyEx(M, imageResult, MORPH_CLOSE, element)
/*	
	int i,j;
	
	printf("CHECK THIS");
	for(i=0;i<240;i++){
		for(j=0;j<520;j++){
			//p_imgResult[j*520+i] = (int)imageResult.data[j*520+i];
			
			printf("%d ",imageResult.at<uint8_t>(i,j));
		}
		printf("\n");
	}
*/
/*
	int i;
	unsigned char *Rbuff = imageResult.data;
	for(i=0; i<240*520;i++){
	Hmat_z[i]= (int)Rbuff[i];
*/

	
	//Mat img(240,520,CV_8UC2,imageResult);
	//p_imgResult = img.data;
	
	//p_imgResult = imageResult.data;
	/*
	int i,j;
	
	printf("CHECK THIS");
	for(j=0;j<240;j++){
		for(i=0;i<520;i++){
			//p_imgResult[j*520+i] = (int)imageResult.data[j*520+i];
			p_imgResult[j*520+i] = imageResult.at<uint8_t>(j,i);
			printf("%d ",p_imgResult[j*520+i]);
		}
		printf("\n");
	}
	return;
	*/

	//size = (n,n);
	//shape = cv2.MORPH_RECT;
	//kernel = cv2.getStructuringElement(shape,size);
	
	//cv2.dilate(M,M,kernal); //Not sure if arrays will work here


//OLD
///////////////////////////////
/*
#include "std.h"
#include "opencv_to_c.h"
#include "modules/computer_vision/lib/vision/image.h"

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
//#include <iostream> //for printing to terminal
using namespace cv;



void imageProcess(struct image_t *image)
{
//If throws an error, then revert back to returning structure pointer, such as in crash course
  int width = img->width;
  int height = img->height;
  struct image_t greyImage;
  
  image_create(greyImage,width,height,IMAGE_INT16);
  image_to_grayscale(image,greyImage);
  
  Mat M(height, width, CV_8UC2, img);
  //Mat imgOLD;
  //imgOLD = imread("../C-code/30315196.jpg"); //get image from file (need to change)
  Mat image;
  Mat mtx;
  Mat dist;
  
  //cvtColor(M, image, CV_YUV2BGR_UYVY);//CV_8UC2 OR CV_YUV2BGR_Y422); //convert to MAT
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
