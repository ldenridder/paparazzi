#include "opencv_to_c.h" //Modify this file
#include <stdio.h>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

void noiseFilter(int *p_hmat_z, int X,int Y){

	int x,y;
	
	int fch = 50;
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
	return;
}

/*
void maximumBoxFilter(int n, int *p_hmat_z){

	int i,j;

	int array[520][240];


	for(i=0;i<240;i++){
		for(j=0;j<520;j++){
			array[j][i] = p_hmat_z[i*520+j];
		}
	}
	

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
*/
