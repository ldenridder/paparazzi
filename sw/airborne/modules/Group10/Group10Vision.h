#ifndef GROUP10VISION_H
#define GROUP10VISION_H

#include "modules/computer_vision/cv.h"


extern int X;

/*
void shape_ind(int *p_xx_grad, int *p_yy_grad, int *p_xy_grad, double *p_shape_index, int X, int Y);
void grad_x(int *p_img, int *p_x_grad, int X, int Y);
void grad_y(int *p_img, int *p_y_grad, int X, int Y);
void hmat_z_func(int *p_img, double *p_shape_index, int *p_hmat_z, float htarget, int X, int Y);
void horizonEstimator(int theta, int phi, int pitchGain, int Y, float *m, float *b);
int HorizonFilter(int *img, float m, float b, int BUFFER, int X, int Y);
int array_find(int *img, int i, int j, int X, int grid_height, int grid_width);
void grid_counter(int *img, int *p_grid, int n_rows, int n_columns, int grid_height, int grid_width, int X);
void output_conversion(int *p_grid, int *p_navInput, int n_columns, int n_rows);
*/

void filter(int *img, double m, int b, double mu_s, double mu_g, int threshold_s, int threshold_g, int X, int Y, int *imgFiltered);
void meanGrey(int *img, double *mu_s, int *n_s, double *mu_g, int *n_g, double m, int b, int X, int Y);
void cov(int *img, double m, int b, int *n_s, double *mu_s, int *n_g, double *mu_g, double *sigma_s, double *sigma_g, int X, int Y);
void findSegmentation(int *img, int X, int Y, double *m, int *b, double *mu_sFINAL, double *mu_gFINAL);

void visionInit(void);
//void visionPeriodic(void);
struct image_t *imageProcess(struct image_t *image);


#endif
