#ifndef GROUP10VISION_H
#define GROUP10VISION_H

#include "modules/computer_vision/cv.h"


extern int X;

void shape_ind(int *p_xx_grad, int *p_yy_grad, int *p_xy_grad, double *p_shape_index, int X, int Y);
void grad_x(int *p_img, int *p_x_grad, int X, int Y);
void grad_y(int *p_img, int *p_y_grad, int X, int Y);
void hmat_z_func(int *p_img, double *p_shape_index, int *p_hmat_z, float htarget, int X, int Y);
void horizonEstimator(int theta, int phi, int pitchGain, int Y, float *m, float *b);
int HorizonFilter(int *img, float m, float b, int BUFFER, int X, int Y);
int array_find(int *img, int i, int j, int X, int grid_height, int grid_width);
void grid_counter(int *img, int *p_grid, int n_rows, int n_columns, int grid_height, int grid_width, int X);
void output_conversion(int *p_grid, int *p_navInput, int n_columns, int n_rows);
void cluster_creator(int *p_img, int X, int Y, int *cluster);
void Check_NB(int i, int j, int *visited, int *p_img, int *running_cluster_ind, int X, int Y, int *cluster, int *recursive_depth_stopper);
int Check_Save(int i, int j, int visited_point, int image_point, int X, int Y);
void cluster_filter(int *cluster, int X, int Y, int *filteredImage);
void visionInit(void);
void green_detect(struct image_t *image, int X, int Y, int *green);
//void visionPeriodic(void);
struct image_t *imageProcess(struct image_t *image);


#endif
