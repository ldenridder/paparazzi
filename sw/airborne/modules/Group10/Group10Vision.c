#include <stdio.h>
#include <math.h>
#include "modules/Group10/Group10Vision.h"
#include "subsystems/abi.h"

#include "modules/computer_vision/lib/vision/image.h"
#include "opencv_to_c.h"

#ifndef PROCESS_FPS
#define PROCESS_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

/*The following funciton is a modification of the main branch. It does not use cpp. All image processing is done here, then the vector of allowable directions is messaged through abi*/
struct image_t *imageProcess(struct image_t *image){
	//printf("Got here one \n");
	//printf("Image address: %p\n", image);
	int X = 520; int Y = 240;
	//printf("Address of X is: %p\n", &X);
	int n_rows = 10; int n_columns = 7;
	int grid_height = Y/n_rows;
	int grid_width = X/n_rows;
	int img[X*Y];
	int x_grad[X*Y], y_grad[X*Y], xx_grad[X*Y], yy_grad[X*Y], xy_grad[X*Y];
	double shape_index[X*Y];
	int hmat_z[X*Y];
	int navInput[7] = {0,0,0,0,0,0,0};
	int grid[n_rows*n_columns];
	int cluster[X*Y];
	int filteredImage[X*Y];

	int i;
	for(i=0;i<(X*Y);i++){
		cluster[i] = -1;
		filteredImage[i] = 0;
		hmat_z[i]=0;
	}
	//printf("Size of cluster: %d\n",(sizeof(cluster)/sizeof(cluster[0])));
	
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
	//hmat_z_func(img,shape_index,hmat_z,0.5,X,Y);
	//hmat_z_func(img,shape_index,hmat_z,0,X,Y);
	
	//printf("hmat_z: \n");
	/*
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",hmat_z[y*X+x]);
		}
		printf("\n");
	}
	*/
	noiseFilter(hmat_z,X,Y);
	//printf("\n hmat_z FILTERED: \n");
	/*
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",hmat_z[y*X+x]);
		}
		printf("\n");
	}
*/

	cluster_creator(hmat_z, X, Y, cluster);
	
	/*
	printf("Cluster: \n");
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",cluster[y*X+x]);
		}
		printf("\n");
	}
*/
	cluster_filter(cluster, X, Y, filteredImage);
	
	/*
	printf("Filtered image: \n");
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",filteredImage[y*X+x]);
		}
		printf("\n");
	}
	*/
	//printf("CLUSTER: \n");
	//for(y=0;y<Y;y++){
	//	for(x=0;x<X;x++){
	//		printf("%d\n ", cluster[y*X+x]);
	//	}
	//	printf("\n");
	//}
	
	grid_counter(filteredImage,grid,n_rows,n_columns,grid_height,grid_width,X);
	/*
	for(y=0;y<n_rows;y++){
		for(x=0;x<n_columns;x++){
			printf("%d ",grid[y*X+x]);
		}
		printf("\n");
	}
	*/
	/*
	for(x=0;x<n_rows;x++){
		printf("%d ",navInput[x]);
	}
	printf("\n");
	*/
	
	output_conversion(grid,navInput,n_columns,n_rows);
	
	for(x=0;x<n_columns;x++){
		printf("%d ",navInput[x]);
	}
	printf("\n");
	/*
	printf("Grid output\n");
	printf("Value: %p\n",navInput);
	printf("Lane1: %d\n",navInput[0]);
	printf("Lane2: %d\n",navInput[1]);
	printf("Lane3: %d\n",navInput[2]);
	*/
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


void visionInit(void){
	//printf("Got here \n");
	cv_add_to_device(&VIDEO_CAPTURE_CAMERA, imageProcess, PROCESS_FPS);
	//printf("Got here again \n");
}







void cluster_creator(int *p_img, int X, int Y, int *cluster)
{
	int visited[X*Y];
	int v;
	for(v=0;v<X*Y;v++){
		visited[v]=0;
}
	int running_cluster_ind=0;
	int i, j;
	//printf("Entered cluster creator \n");
	for(i=0;i<Y;i++)
	{
		for(j=0;j<X;j++)
		{
		if(visited[i*X+j] == 0 && p_img[i*X+j]==1){Check_NB(i, j, visited, p_img, &running_cluster_ind, X, Y, cluster);} 
		running_cluster_ind +=1;
		}
	}
}

void Check_NB(int i, int j, int *visited, int *p_img, int *running_cluster_ind, int X, int Y, int *cluster) 
{	
	//int a;
	//printf("First 10 of cluster: \n");
	//for(a=0;a<10;a++){
	//	printf("%d ",cluster[a]);
	//}
	//printf("\n");
	
	
	int k; 
	int neighb_i[8]={-1,-1,-1,0,0,1,1,1};
	int neighb_j[8]={-1,0,+1,-1,1,-1,0,1};

	//printf("Check NB \n");

	visited[i*X + j]= 1;
	//printf("Visited\n");
	//printf("Cluster(running_cluster_ind) = %d\n",cluster[(*running_cluster_ind)]);
	cluster[*running_cluster_ind] = i*X + j;
	//printf("Cluster\n");
	*running_cluster_ind += 1;
	
	//printf("Running cluster ind: %d\n",(*running_cluster_ind));
	for(k=0; k<8; k++) 
	{	
		int nbp_i = i+neighb_i[k];
		int nbp_j = j+neighb_j[k];
		if(Check_Save(nbp_i, nbp_j, visited[(nbp_i)*X + nbp_j], p_img[(nbp_i)*X + nbp_j], X, Y)==1){
			Check_NB(nbp_i, nbp_j,  visited, p_img, running_cluster_ind, X, Y, cluster);
		} 
	}
	return;
}

int Check_Save(int i, int j, int visited_point, int image_point, int X, int Y){
	int return_result=0;
	int i_cond_0=0;
	int j_cond_0=0;
	int i_cond_end_row=0;
	int j_cond_end_col=0;
	int entry_visited=0;
	int image_point_cond=0;
	/* couple of checks */

	//printf("boolean value of i = %d\n", i);
	//printf("Check save \n");
	if(0<=i && i<Y && 0<=j && j<X){
		i_cond_end_row=1;
		j_cond_0=1;
		j_cond_end_col=1;
		i_cond_0 = 1;
		if(visited_point==0 && image_point==1){
			entry_visited=1;
			image_point_cond=1;}
		}
	//printf("i_cond_0 value of i = %d\n", i_cond_0);	
	/*

else if(i<Y){i_cond_end_row=1;}


	if(0<=j){j_cond_0=1;}
	else if(j<X){j_cond_end_col=1;}

	else if(visited_point==0){entry_visited=1;}

	else if(image_point==1){image_point_cond=1;}
*/
	int summed_condition = i_cond_0 + j_cond_0 + i_cond_end_row + j_cond_end_col + entry_visited + image_point_cond;
	//printf("summed condition: %d\n ",summed_condition );
	if(summed_condition==6){return_result = 1;}
	return return_result;
}


void cluster_filter(int *cluster, int X, int Y, int *filteredImage){
	int i;
	int j;
	int size_object=0;
	//printf("Check cluster filt \n");
	
	int firstIndex = -1;
	for(i=0;i<(X*Y);i++){
		if(cluster[i] != -1 && firstIndex == -1){
			firstIndex = i;
		}
		else if(cluster[i] == -1 && firstIndex != -1){
			if((i-firstIndex) < 85){
				for(j=firstIndex;j<i;j++){
					cluster[j] = -1;
				}
				firstIndex = -1;
			}
		}
	}
	int index;
	for(i=0;i<(X*Y);i++){
		if(cluster[i] != -1){
			//printf("Entered here\n");
			index = cluster[i];
			//printf("Index = %d\n",index);
			filteredImage[index] = 1;
		}
	}
	return;
}

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
				//printf("%d ",p_hmat_z[y*X+x]);
			}
		}
		//printf("\n");
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
