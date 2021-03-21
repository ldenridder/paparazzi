#include <stdio.h>
#include <math.h>
#include "Group10Vision.h"
#include "subsystems/abi.h"

#ifndef PROCESS_FPS
#define PROCESS_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

void avoiderInit(void){
	cv_add_to_device(&VIDEO_CAPTURE_CAMERA, imageProcess, PROCESS_FPS);
}

void avoiderPeriodic(void){
	int X = 520;
	int Y = 240;
	int n_rows = 10;
	int n_columns = 10;
	int grid_height = Y/n_rows;
	int grid_width = X/n_rows;
	int x_grad[X*Y], y_grad[X*Y], xx_grad[X*Y], yy_grad[X*Y], xy_grad[X*Y];
	int shape_index[X*Y];
	int hmat_z[X*Y];
	int navInput[X];
	
	grad_x(img,x_grad,X,Y);
	grad_y(img,y_grad,X,Y);
	grad_x(x_grad,xx_grad,X,Y);
	grad_y(y_grad,yy_grad,X,Y);
	grad_y(x_grad,xy_grad,X,Y);
	shape_ind(xx_grad,yy_grad,xy_grad,shape_index,X,Y);
	hmat_z_func(img,shape_index,hmat_z,-0.5,X,Y);
	hmat_z_func(img,shape_index,hmat_z,0.5,X,Y);
	hmat_z_func(img,shape_index,hmat_z,0,X,Y);
	noiseFilter(hmat_z,X,Y);
	grid_counter(img,grid,n_rows,n_columns,grid_height,grid_width,X);
	output_conversion(grid,navInput,n_columns,n_rows);
	
	AbiSendMsgNAVIGATION_VECTOR(NAVIGATION_VECTOR_ID,navInput);
	
}

//Helper functions


void shape_ind(int *p_xx_grad, int *p_yy_grad, int *p_xy_grad, double *p_shape_index, int X, int Y)
{
//Check which variables actually need to be arrays
	int i, j;
	double T, D, L1, L2;
	double Lk1, Lk2;
	double Ldiff, Ladds;
	for(i=0;i<Y;i++)
	{
		for(j=0;j<X;j++)
		{
			T = p_xx_grad[i*X+j] + p_yy_grad[i*X+j];
			D = (p_xx_grad[i*X+j])*(p_yy_grad[i*X+j]) - (p_xy_grad[i*X+j])*(p_xy_grad[i*X+j]);
			L1 = 0.5*T + sqrt((T*T)/4 - D);
			L2 = 0.5*T - sqrt((T*T)/4 - D);
			if(L1>L2){Lk1 = L1; Lk2 = L2;}
			else{Lk1 = L2; Lk2 = L1;}
			Ldiff = Lk2 - Lk1;
			Ladds = Lk2 + Lk1;
			p_shape_index[i*X+j] = (2/M_PI)*atan(Ladds/Ldiff);
		}
	}
	return;
}

void grad_x(int *p_img, int *p_x_grad, int X, int Y)
{
	int i, j;
	for(j=0;j<Y;j++)
	{
		for(i=0;i<X;i++)
		{
			if(i==0)
			{
				p_x_grad[j*X+i] = (p_img[j*X+i+1])-(p_img[j*X+i]);
			}
			else if(i==(X-1))
			{
				p_x_grad[j*X+i] = (p_img[j*X+i])-(p_img[j*X+i-1]);
			}
			else
			{
				p_x_grad[j*X+i] = ((p_img[j*X+i+1])-(p_img[j*X+i-1]))/2;
			}
		}
	}
	return;
}

void grad_y(int *p_img, int *p_y_grad, int X, int Y)
{
	int i, j;
	for(j=0;j<Y;j++)
	{
		for(i=0;i<X;i++)
		{
			if(j==0)
			{
				p_y_grad[j*X+i] = (p_img[(j+1)*X+i]) - (p_img[j*X+i]);
			}
			else if(j==(Y-1))
			{
				p_y_grad[j*X+i] = (p_img[j*X+i]) - (p_img[(j-1)*X+i]);
			}
			else
			{
				p_y_grad[j*X+i] = ((p_img[(j+1)*X+i]) - (p_img[(j-1)*X+i]))/2;
			}
		}
	}
}

void hmat_z_func(int *p_img, double *p_shape_index, int *p_hmat_z, float htarget, int X, int Y){
	float hdelta = 0.05;
	int x, y;
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			if(((p_shape_index[y*X+x] - htarget)<hdelta) && (p_img[y*X+x] >= 100)){
				p_hmat_z[y*X+x] += p_img[y*X+x];
			}
		}
	}
	return;
}

/*
void maximumBoxFilter(int *p_hmat_z, int n){
	
}

void noise_filter(int *p_hmat_z){
	
}
*/

/* The following function estimates the horizon based on the pitch and roll angles */
void horizonEstimator(int theta, int phi, int pitchGain, int Y, float *m, float *b)
{
	*b = (Y/2) - pitchGain*theta;
	*m = tan(phi*(M_PI/180.0));
	return;
}

/* The following function returns 1 if an object is completely above or below the horizon, meaning it should be filtered out. Includes a buffer zone */
//img here denotes the array for one object
int HorizonFilter(int *img, float m, float b, int BUFFER, int X, int Y)
{
	//Needs to be checked
	int i, j;
	int xMin = 1000, yMin = 1000, xMax = -1, yMax = -1;
	int y1,y2,y3,y4,x1,x2,x3,x4;
	for(i = 0; i < Y; i++)
	{
		for(j = 0; j < X; j++)
		{
			if(img[i*X + j] != 0)
			{
				if(i > yMax){yMax = i;}
				if(i < yMin){yMin = i;}
				if(j > xMax){xMax = j;}
				if(j < xMin){xMin = j;} 
			}
		}
	}
	y1 = y2 = yMin;
	y3 = y4 = yMax;
	x1 = x3 = xMin;
	x2 = x4 = xMax;
	
	if((y1 < (m*x1 + b - BUFFER)) && (y2 < (m*x2 + b - BUFFER)) && (y3 < (m*x3 + b - BUFFER)) && (y4 < (m*x4 + b - BUFFER)))
	{
		return 1; //Filter out object (true)
	}
	else if((y1 > (m*x1 + b + BUFFER)) && (y2 > (m*x2 + b + BUFFER)) && (y3 > (m*x3 + b + BUFFER)) && (y4 > (m*x4 + b + BUFFER)))
	{
		return 1; //Filter out object (true)
	}
	else {return 0;} //Keep object (false)
}

//img here is for the entire image, after filtering out some objects
int array_find(int *img, int i, int j, int X, int grid_height, int grid_width)
{
	int k, l;
	int sum = 0;
	for(k=(i*grid_height); k<((i+1)*grid_height); k++)
	{
		for(l=(j*grid_width); l<((j+1)*grid_width); l++)
		{
			if(img[k*X + l] > 0)
			{
				return 1;
			}
		}
	}
	return 0;
}

//img here is for the entire image, after filtering out some objects (240x520)
//grid is (9x7)
//n_rows number of rows in the grid
//n_columns number of columns in the grid
void grid_counter(int *img, int *p_grid, int n_rows, int n_columns, int grid_height, int grid_width, int X)
{
	//img here is after filtering, so all objects remaining are compiled into one image and need to be avoided
	//grid_height and grid_width are the height and width of each square in the grid
	int i, j;
	
	for(i=0; i<n_rows; i++)
	{
		for(j=0; j<n_columns; j++)
		{
			p_grid[i*n_columns+j] = array_find(img, i, j, X, grid_height, grid_width);
		}
	}
	return;
}

void output_conversion(int *p_grid, int *p_navInput, int n_columns, int n_rows)
{
	int i, j;
	for(i=0;i<n_columns;i++)
	{
		for(j=n_rows-1;j>=0;j--)
		{
			if(p_grid[j*n_columns + i] == 0)
			{
				p_navInput[i]++;
			}
			else
			{
				break;
			}
		}
	}
	return;
}
