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

	int X = 520; int Y = 240;
	int n_rows = 10; int n_columns = 7;
	int grid_height = Y/n_rows;
	int grid_width = X/n_columns;
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
	int green[4] = {0,0,0,0};

	//printf("check if here \n");
	green_detect(image,X,Y,green);
	printf("check if here past green detection \n");
	int green1 = green[0];
	int green2 = green[1];
	int green3 = green[2];
	int green4 = green[3];
    	printf("green: %d,%d,%d,%d \n",green1,green2,green3,green4);

	//Convert image to greyscale
	image_to_grayscale(image,image);

	uint8_t *imageValues = image->buf;

	
	//Convert image to array, keeping in mind that we need to transpose it.
	int x; int y;
	int k = 1;
	/*
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			img[y*X+x] = imageValues[k];
			k += 2;
		}
	}
	*/
	for(x=0;x<X;x++){
		for(y=(Y-1);y>=0;y--){
			img[y*X+x] = imageValues[k];
			k += 2;
		}
	}

	
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

	

	int hmat_z_new[X*Y];
/*
	printf("before noise fileter: \n");
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",hmat_z[y*X+x]);
		}
		printf("\n");
	}
*/
	noiseFilter(hmat_z, hmat_z_new, X,Y);
	/*
	printf("after noise fileter: \n");
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",hmat_z[y*X+x]);
		}
		printf("\n");
	}
	
*/
	cluster_creator(hmat_z, X, Y, cluster);

	cluster_filter(cluster, X, Y, filteredImage);
/*
	printf("filteredImage: \n");
	for(y=0;y<Y;y++){
		for(x=0;x<X;x++){
			printf("%d ",filteredImage[y*X+x]);
		}
		printf("\n");
	}
*/
	grid_counter(filteredImage,grid,n_rows,n_columns,grid_height,grid_width,X);

	printf("GRID: \n");
	for(y=0;y<n_rows;y++){
		for(x=0;x<n_columns;x++){
			printf("%d ",grid[y*n_columns+x]);
		}
		printf("\n");
	}
	
	
	output_conversion(grid,navInput,n_columns,n_rows);
	
	printf("NAV INPUT: \n");
	for(x=0;x<n_columns;x++){
		printf("%d ",navInput[x]);
	}
	printf("\n");

	int allowableDistance1 = navInput[0];
	int allowableDistance2 = navInput[1];
	int allowableDistance3 = navInput[2];
	int allowableDistance4 = navInput[3];
	int allowableDistance5 = navInput[4];
	int allowableDistance6 = navInput[5];
	int allowableDistance7 = navInput[6];

	AbiSendMsgNAVIGATION_VECTOR(NAVIGATION_VECTOR_ID,allowableDistance1,allowableDistance2,allowableDistance3,allowableDistance4,allowableDistance5,allowableDistance6,allowableDistance7,green1,green2,green3,green4);

	return image;
}


void green_detect(struct image_t *image, int X, int Y, int *green)
{
	  int y_min = 50; //60
	  int y_max = 120; //150
	  int u_min = 70; //70
	  int u_max = 90; //120
	  int v_min = 110; // 110
	  int v_max = 140; //140
	  int a = 0;
	  int b = 0;
	  int c = 0;
		
	  uint8_t *buf = image->buf;
	  buf ++;
	  //printf("Test");
	  for (int x=0;x<X;x++){
	    for (int y=0;y<Y;y++){

	    	if(
				(buf[1] >= y_min)
				&& (buf[1] <= y_max)
				&& (buf[0] >= u_min)
				&& (buf[0] <= u_max)
				&& (buf[2] >= v_min)
				&& (buf[2] <= v_max)
			  ){
				a = x - X/2;
				b = y - Y/2;
				c = 2 * ((a >= 0) - (a < 0)) + ((b >= 0) - (b < 0));
				//printf("right before switch \n");
				switch (c){
				  case  3: //4
					green[3]++;
					//printf("X,Y %d, %d", x,y);
					break;
				  case -3: //1
					green[0]++;
					//printf("X,Y %d, %d", x,y);
					break;
				  case -1: //3
					green[1]++;
					break;
				  case  1: //2
					green[2]++;
					break;
				  default:
					break;
				}
				//printf("we zijn tot hier gekomen \n");
				//break;
	    		}
		//printf("maar nu zijn we hier %d \n", x);
		//printf("maar nu zijn we hier %d \n", y);
	    	buf += 3; // each pixel has two bytes
		//printf("crashed die hier? %d \n", buf);
	    }
		//printf("buiten de y loop %d \n", x);
	  }
	  //printf("Test3");

	  return;
}


void visionInit(void){
	cv_add_to_device(&VIDEO_CAPTURE_CAMERA, imageProcess, PROCESS_FPS);

}







void cluster_creator(int *p_img, int X, int Y, int *cluster)
{
	int visited[X*Y];
	int v;
	for(v=0;v<X*Y;v++){
		visited[v]=0;
}
	int running_cluster_ind=0;
	int recursive_depth_stopper=0;
	int i, j;
	for(i=0;i<Y;i++)
	{
		for(j=0;j<X;j++)
		{
		if(visited[i*X+j] == 0 && p_img[i*X+j]==1){Check_NB(i, j, visited, p_img, &running_cluster_ind, X, Y, cluster, &recursive_depth_stopper);} 
		running_cluster_ind +=1;
		recursive_depth_stopper =0;
		}
	}
}

void Check_NB(int i, int j, int *visited, int *p_img, int *running_cluster_ind, int X, int Y, int *cluster, int *recursive_depth_stopper) 
{	
	
	int k; 
	int neighb_i[8]={-1,-1,-1,0,0,1,1,1};	//{-1,0,0,1};
	int neighb_j[8]={-1,0,+1,-1,1,-1,0,1}; //{0,-1,1,0};

	visited[i*X + j]= 1;

	cluster[*running_cluster_ind] = i*X + j;

	*running_cluster_ind += 1;
	*recursive_depth_stopper +=1;
	//printf("recursive_depth_stopper %d \n", *recursive_depth_stopper);
	for(k=0; k<8; k++) 
	{	
		int nbp_i = i+neighb_i[k];
		int nbp_j = j+neighb_j[k];
		if(Check_Save(nbp_i, nbp_j, visited[(nbp_i)*X + nbp_j], p_img[(nbp_i)*X + nbp_j], X, Y)==1 && *recursive_depth_stopper<3000){
			Check_NB(nbp_i, nbp_j,  visited, p_img, running_cluster_ind, X, Y, cluster, recursive_depth_stopper);
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

	if(0<=i && i<Y && 0<=j && j<X){
		i_cond_end_row=1;
		j_cond_0=1;
		j_cond_end_col=1;
		i_cond_0 = 1;
		if(visited_point==0 && image_point==1){
			entry_visited=1;
			image_point_cond=1;}
		}
	int summed_condition = i_cond_0 + j_cond_0 + i_cond_end_row + j_cond_end_col + entry_visited + image_point_cond;
	if(summed_condition==6){return_result = 1;}
	return return_result;
}


void cluster_filter(int *cluster, int X, int Y, int *filteredImage){
	int i;
	int j;
	int size_object=0;
	
	int firstIndex = -1;
	for(i=0;i<(X*Y);i++){
		if(cluster[i] != -1 && firstIndex == -1){
			firstIndex = i;
		}
		else if(cluster[i] == -1 && firstIndex != -1){
			if((i-firstIndex) < 2000){
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
			index = cluster[i];
			filteredImage[index] = 1;
		}
	}
	return;
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
			if(L1<L2){Lk1 = L1; Lk2 = L2;}
			else{Lk1 = L2; Lk2 = L1;}
			Ldiff = Lk2 - Lk1;
			Ladds = Lk2 + Lk1;
			if(Ldiff==0){
				if(Ladds<0){
					p_shape_index[i*X+j] = -1;
					}
				else if(Ladds>0){
					p_shape_index[i*X+j] = 1;
					}
}			
			else{

			p_shape_index[i*X+j] = (2/M_PI)*atan(Ladds/Ldiff);}

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
			if((fabs(p_shape_index[y*X+x] - htarget)<hdelta) && (p_img[y*X+x] >= 100)){
				p_hmat_z[y*X+x] += p_img[y*X+x];
			}
		}
	}
	return;
}

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
				sum++;
				if(sum>200){
				return 1;}
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
