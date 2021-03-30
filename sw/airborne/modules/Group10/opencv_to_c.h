#ifndef OPENCV_TO_C_H
#define OPENCV_TO_C_H

#ifdef __cplusplus
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/calib3d.hpp>
//using namespace cv;
extern "C" 
{

#endif
  
  //int imageProcess(char *img, int width, int height);
  void noiseFilter(int *p_hmat_z, int X,int Y);
  void maximumBoxFilter(int n, int *p_hmat_z);
  //void getImage();
  //int getDist();
  //int getMtx();

#ifdef __cplusplus

}
#endif

#endif

