extern int getMaximum(int * Im);
extern int getMinimum(int * Im);
extern void getGradientPixelWH(unsigned char *frame_buf, int x, int y, int* dx, int* dy);
extern void getSimpleGradient(unsigned char* frame_buf, int* DX, int* DY);
extern void multiplyImages(int* ImA, int* ImB, int* ImC, int width, int height);
extern void getImageDifference(int* ImA, int* ImB, int* ImC, int width, int height);
extern int calculateError(int* ImC, int width, int height);
extern void printIntMatrix(int* Matrix, int width, int height);
extern void printIntMatrixPart(int* Matrix, int width, int height, int n_cols, int n_rows, int x_center, int y_center);
extern void smoothGaussian(int* Im);
extern void getHarris(int* DXX, int* DXY, int* DYY, int* Harris);
extern int findLocalMaxima(int* Harris, int max_val, int MAX_POINTS, int* p_x, int* p_y, int suppression_distance_squared, int* n_found_points);
extern void excludeArea(unsigned int* Mask, int x, int y, int suppression_distance_squared);
extern void thresholdImage(int* Harris, int max_val, int max_factor);
extern int findCorners(unsigned char *frame_buf, int MAX_POINTS, int *x, int *y, int suppression_distance_squared, int* n_found_points, int mark_points, int imW, int imH);
extern void getSubPixel(int* Patch, unsigned char* buf, int center_x, int center_y, int half_window_size, int subpixel_factor);
extern int calculateG(int* G, int* DX, int* DY, int half_window_size);
extern void getGradientPatch(int* Patch, int* DX, int* DY, int half_window_size);
extern int getSumPatch(int* Patch, int size);
extern void showFlow(unsigned char * frame_buf, int* x, int* y, int* status, int n_found_points, int* new_x, int* new_y, int imgW, int imgH);
extern int opticFlowLK(unsigned char * new_image_buf, unsigned char * old_image_buf, int* p_x, int* p_y, int n_found_points, int imW, int imH, int* new_x, int* new_y, int* status, int half_window_size, int max_iterations);
