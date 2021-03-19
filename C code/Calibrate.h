#ifndef CALIBRATE_H
#define CALIBRATE_H

//#include <utility> //For the std::pair return, not needed if just changing the private variables

// #include <opencv2> //Not needed in header

class Calibrate
{
private:
    int img;
    int mtx;
    int dist;
 
public:
    Calibrate(int img, int mtx, int dist);

    void CalibrateCam();
    
    int Undistort(int image);

    int getMtx() { return mtx;}
    int getDist() { return dst;}
};
 
#endif