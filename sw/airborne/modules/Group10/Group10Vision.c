#include <stdio.h>
#include <math.h>
#include "modules/Group10/Group10Vision.h"
#include "subsystems/abi.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "opencv_to_c.h"

#ifndef PROCESS_FPS
#define PROCESS_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

struct image_t *imageProcess(struct image_t *image){
	int X = 520; int Y = 240;
	int n_rows = 7; int n_columns = 10;
	int grid_height = Y/n_rows;
	int grid_width = X/n_rows;
	int img[X*Y];
	int navInput[7] = {0,0,0,0,0,0,0};
	int grid[n_rows*n_columns];
	int imgFiltered[X*Y];
	double m;
	int b;
	double mu_s, mu_g;
	int threshold_s = 3;
	int threshold_g = 3;
	
	convertToGrey((char *) image->buf, image->w, image->h, img);
	
	/*
	int x,y;
	printf("IMAGE: \n");
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",img[y*X+x]);
		}
		printf("\n");
	}
	*/

	
	findSegmentation(img, X, Y, &m, &b, &mu_s, &mu_g);
	//printf("m = %d\n",m);
	//printf("b = %d\n",b);
	filter(img, m, b, mu_s, mu_g, threshold_s, threshold_g, X, Y, imgFiltered);
	
	/*
	int x,y;
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",imgFiltered[y*X+x]);
		}
		printf("\n");
	}
	*/
	
	int allowableDistance1 = navInput[0];
	int allowableDistance2 = navInput[1];
	int allowableDistance3 = navInput[2];
	int allowableDistance4 = navInput[3];
	int allowableDistance5 = navInput[4];
	int allowableDistance6 = navInput[5];
	int allowableDistance7 = navInput[6];

AbiSendMsgNAVIGATION_VECTOR(NAVIGATION_VECTOR_ID,allowableDistance1,allowableDistance2,allowableDistance3,allowableDistance4,allowableDistance5,allowableDistance6,allowableDistance7);
	//printf("Abi messaging out\n");
	return image;
}

void visionInit(void){
	//printf("Got here \n");
	cv_add_to_device(&VIDEO_CAPTURE_CAMERA, imageProcess, PROCESS_FPS);
	//printf("Got here again \n");
}

void filter(int *img, double m, int b, double mu_s, double mu_g, int threshold_s, int threshold_g, int X, int Y, int *imgFiltered){
	int x,y;
	
	if(mu_s == 0){ //All ground
		double ground_low = mu_g - threshold_g;
		double ground_high = mu_g + threshold_g;
		
		for(y=0;y<Y;y++){
			for(x=0;x<X;x++){
				if((img[y*X+x] > ground_high) || (img[y*X+x] < ground_low)){
						imgFiltered[y*X+x] = 1;
					}
				else{
					imgFiltered[y*X+x] = 0;
				}
			}
		}
	}
	else if(mu_g == 0){ //All sky
		double sky_low = mu_s - threshold_s;
		double sky_high = mu_s + threshold_s;
		
		for(y=0;y<Y;y++){
			for(x=0;x<X;x++){
				if(y<m*x+b){ //Sky
					if((img[y*X+x] > sky_high) || (img[y*X+x] < sky_low)){
						imgFiltered[y*X+x] = 1;
					}
					else{
						imgFiltered[y*X+x] = 0;
					}
				}
			}
		}
	}
	else{
		double sky_low = mu_s - threshold_s;
		double sky_high = mu_s + threshold_s;
		double ground_low = mu_g - threshold_g;
		double ground_high = mu_g + threshold_g;
		
		for(y=0;y<Y;y++){
			for(x=0;x<X;x++){
				if(y<m*x+b){ //Sky
					if((img[y*X+x] > sky_high) || (img[y*X+x] < sky_low)){
						imgFiltered[y*X+x] = 1;
					}
					else{
						imgFiltered[y*X+x] = 0;
					}
				}
				else{ //Ground
					if((img[y*X+x] > ground_high) || (img[y*X+x] < ground_low)){
						imgFiltered[y*X+x] = 1;
					}
					else{
						imgFiltered[y*X+x] = 0;
					}
				}
			}
		}
	}
	return;
}

void meanGrey(int *img, double *mu_s, int *n_s, double *mu_g, int *n_g, double m, int b, int X, int Y){
	int x, y;
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			//printf("m = %d\n",m);
			//printf("b = %d\n",b);
			if(y<(m*x+b)){
				//Pixel is in sky segment for this 'test' segmentation line
				(*n_s)++;
				//printf("n_s = %d\n",*n_s);
				(*mu_s) += img[y*X+x];
				//printf("mu_s = %f\n", (*mu_s));
			}
			else{
				//Pixel is in ground segment for this 'test' segmentation line
				(*n_g)++;
				(*mu_g) += img[y*X+x];
				//printf("mu_g = %f\n", (*mu_g));
			}
		}
	}
	if(*n_s == 0){*mu_s = 0;}
	else{*mu_s = *mu_s/(*n_s);}
	if(*n_g ==0){*mu_g = 0;}
	else{*mu_g = *mu_g/(*n_g);}
	printf("n_s = %d\n",(*n_s));
	printf("n_g = %d\n",(*n_s));
	printf("mu_s = %f\n", (*mu_s));
	printf("mu_g = %f\n", (*mu_g));
	return;
}

/*
void outerProduct(double *vec, double *output){
	int i;
	output[0] += (vec[0])*(vec[0]);
	output[1] += (vec[0])*(vec[1]);
	output[2] += (vec[0])*(vec[2]);
	output[3] += (vec[1])*(vec[0]);
	output[4] += (vec[1])*(vec[1]);
	output[5] += (vec[1])*(vec[2]);
	output[6] += (vec[2])*(vec[0]);
	output[7] += (vec[2])*(vec[1]);
	output[8] += (vec[2])*(vec[2]);
	return;
}
*/

void cov(int *img, double m, int b, int *n_s, double *mu_s, int *n_g, double *mu_g, double *sigma_s, double *sigma_g, int X, int Y){
	int x, y;
	double diff;
	double sigma_s_temp = 0;
	double sigma_g_temp = 0;
	
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			if(y<m*x+b){
				diff = img[y*X+x] - (*mu_s);
				sigma_s_temp += diff*diff;
			}
			else{
				diff = img[y*X+x] - (*mu_g);
				sigma_g_temp += diff*diff;
			}
		}
	}
	if(*n_s <= 1){
		*sigma_g = sigma_g_temp/(*n_g - 1);
		*sigma_s = 0;
	}
	else if(*n_g <= 1){
		*sigma_s = sigma_s_temp/(*n_s - 1);
		*sigma_g = 0;
	}
	else{
		*sigma_s = sigma_s_temp/(*n_s - 1);
		*sigma_g = sigma_g_temp/(*n_s - 1);
	}
	return;
}

/*
void det(double *sigma_s, double *sigma_g, double *det_s, double *det_g){
	*det_s = sigma_s[0]*(sigma_s[4] * sigma_s[8] - sigma_s[5] * sigma_s[7]) - sigma_s[1]*(sigma_s[3] * sigma_s[8] - sigma_s[5] * sigma_s[6]) + sigma_s[2] * (sigma_s[3] * sigma_s[7] - sigma_s[4] * sigma_s[6]);
	
	*det_g = sigma_g[0]*(sigma_g[4] * sigma_g[8] - sigma_g[5] * sigma_g[7]) - sigma_g[1]*(sigma_g[3] * sigma_g[8] - sigma_g[5] * sigma_g[6]) + sigma_g[2] * (sigma_g[3] * sigma_g[7] - sigma_g[4] * sigma_g[6]);
	
	return;
}
*/

void findSegmentation(int *img, int X, int Y, double *m, int *b, double *mu_sFINAL, double *mu_gFINAL){
	double minM = -5;
	double maxM = 5;
	double dM = 0.5;
	int minB = 0;
	int maxB = Y;
	int dB = 1;
	double tempM; int tempB;
	double sigma_s;
	double sigma_g;
	//double det_s, det_g;
	double J1;
	double Jmax = 0;
	
	int n_s, n_g;
	double mu_s, mu_g;
	
	tempM = 0;
	//for(tempM=minM;tempM<=maxM;tempM+=dM){
		for(tempB=minB;tempB<=maxB;tempB+=dB){
			//printf("tempB = %d\n",tempB);
			//printf("tempM = %d\n",tempM);
			meanGrey(img, &mu_s, &n_s, &mu_g, &n_g, tempM, tempB, X, Y);
			//printf("n_s = %d\n",n_s);
			//printf("n_g = %d\n",n_g);
			if(n_s == 0 || n_g == 0){/*printf("Two \n");*/continue;}
			
			cov(img, tempM, tempB, &n_s, &mu_s, &n_g, &mu_g, &sigma_s, &sigma_g, X, Y);
			//det(sigma_s, sigma_g, &det_s, &det_g);
			J1 = 1/(sigma_s + sigma_g);
			if(J1 > Jmax){
				//printf("m = %f\n",tempM);
				//printf("b = %d\n",tempB);
				*m = tempM;
				*b = tempB;
				//printf("tempM = %d\n",tempM);
				//printf("tempB = %d\n",tempB);
				Jmax = J1;
				*mu_sFINAL = mu_s;
				*mu_gFINAL = mu_g;
				//printf("mu_s = %f\n",mu_s);
				//printf("mu_g = %d\n",mu_g);
				//printf("sigma_s = %f\n",sigma_s);
				//printf("sigma_g = %f\n",sigma_g);
			}
		}
	//}
	/*
	printf("m = %f\n",*m);
	printf("b = %d\n",*b);
	printf("mu_s = %f\n",*mu_sFINAL);
	printf("mu_g = %f\n",*mu_gFINAL);
	*/
	return;
}

//////////////////////////////////////////////////////
//OLD CODE
/////////////////////////////////////////////////////
/*The following funciton is a modification of the main branch. It does not use cpp. All image processing is done here, then the vector of allowable directions is messaged through abi*/

/*
struct image_t *imageProcess(struct image_t *image){
	//printf("Got here one \n");
	//printf("Image address: %p\n", image);
	int X = 520; int Y = 240;
	//printf("Address of X is: %p\n", &X);
	int n_rows = 7; int n_columns = 10;
	int grid_height = Y/n_rows;
	int grid_width = X/n_rows;
	int img[X*Y];
	int x_grad[X*Y], y_grad[X*Y], xx_grad[X*Y], yy_grad[X*Y], xy_grad[X*Y];
	double shape_index[X*Y];
	int hmat_z[X*Y];
	int navInput[7] = {0,0,0,0,0,0,0};
	int grid[n_rows*n_columns];
	
	//Convert image to greyscale
	//printf("Got before greyscale \n");
	image_to_grayscale(image,image);
	//printf("Greyscale Image address: %p\n", image);
	//printf("Got after greyscale \n");
	uint8_t *imageValues = image->buf;
	//printf("Got imageValues \n");
	//uint8_t *source = image->buf;
	//printf("Test: %d\n",source[0]);
	//printf("Test2: %d\n",imageValues[0]);
	
	//Convert image to array
	int x; int y;
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			img[y*X+x] = imageValues[y*X+x];
			//printf("%d ",img[y*X+x]);
		}
		//printf("\n");
	}
	//printf("img is: %p\n", img);
	//printf("Got image array\n");
	
	//Perform object detection
	grad_x(img,x_grad,X,Y);
	grad_y(img,y_grad,X,Y);
	grad_x(x_grad,xx_grad,X,Y);
	grad_y(y_grad,yy_grad,X,Y);
	grad_y(x_grad,xy_grad,X,Y);
	shape_ind(xx_grad,yy_grad,xy_grad,shape_index,X,Y);
	hmat_z_func(img,shape_index,hmat_z,-0.5,X,Y);
	hmat_z_func(img,shape_index,hmat_z,0.5,X,Y);
	hmat_z_func(img,shape_index,hmat_z,0,X,Y);
	printf("Object detection \n");
	noiseFilter(hmat_z,X,Y);
	grid_counter(img,grid,n_rows,n_columns,grid_height,grid_width,X);

	for(y=0;y<n_rows;y++){
		for(x=0;x<n_columns;x++){
			printf("%d ",grid[y*X+x]);
		}
		printf("\n");
	}
	

	for(x=0;x<n_rows;x++){
		printf("%d ",navInput[x]);
	}
	printf("\n");


	output_conversion(grid,navInput,n_columns,n_rows);
	
	for(x=0;x<n_rows;x++){
		printf("%d ",navInput[x]);
	}
	printf("\n");

	printf("Grid output\n");
	printf("Value: %p\n",navInput);
	printf("Lane1: %d\n",navInput[0]);
	printf("Lane2: %d\n",navInput[1]);
	printf("Lane3: %d\n",navInput[2]);

	//printf("Lane4: %d\n",navInput[3]);
	//int *navInputPointer = navInput;
	//printf("Array: %p\n",navInput);
	//printf("Pointer: %p\n",navInputPointer);
	//printf("Address of array[0]: %p\n", &navInput);

	int allowableDistance1 = navInput[0];
	int allowableDistance2 = navInput[1];
	int allowableDistance3 = navInput[2];
	int allowableDistance4 = navInput[3];
	int allowableDistance5 = navInput[4];
	int allowableDistance6 = navInput[5];
	int allowableDistance7 = navInput[6];

	AbiSendMsgNAVIGATION_VECTOR(NAVIGATION_VECTOR_ID,allowableDistance1,allowableDistance2,allowableDistance3,allowableDistance4,allowableDistance5,allowableDistance6,allowableDistance7);
	//printf("Abi messaging out\n");
	return image;
}
*/

/*

void visionInit(void){
	//printf("Got here \n");
	cv_add_to_device(&VIDEO_CAPTURE_CAMERA, imageProcess, PROCESS_FPS);
	//printf("Got here again \n");
}
*/

/*
//Need to get rid of visionPeriodic
void visionPeriodic(void){

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
*/



/* The following function estimates the horizon based on the pitch and roll angles */
/*
void horizonEstimator(int theta, int phi, int pitchGain, int Y, float *m, float *b)
{
	*b = (Y/2) - pitchGain*theta;
	*m = tan(phi*(M_PI/180.0));
	return;
}
*/
/* The following function returns 1 if an object is completely above or below the horizon, meaning it should be filtered out. Includes a buffer zone */
//img here denotes the array for one object
/*
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
*/
//img here is for the entire image, after filtering out some objects
/*
int array_find(int *img, int i, int j, int X, int grid_height, int grid_width)
{
	int k, l;
	//int sum = 0;
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
*/

//img here is for the entire image, after filtering out some objects (240x520)
//grid is (9x7)
//n_rows number of rows in the grid
//n_columns number of columns in the grid
/*
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
*/
/*
void output_conversion(int *p_grid, int *p_navInput, int n_columns, int n_rows)
{
	int i, j;
	
	for(j=0;j<n_rows;j++){
		for(i=0;i<n_columns;i++){
			printf("%d ",p_grid[j*n_columns+i]);
		}
		printf("\n");
	}
	for(i=0;i<n_rows;i++){
		printf("%d ",p_navInput[i]);
	}
	printf("\n");
	
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
*/
