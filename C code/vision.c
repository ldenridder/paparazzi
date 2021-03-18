#include <stdio.h>
#include <math.h>


/* The following function estimates the horizon based on the pitch and roll angles */
void horizonEstimator(int theta, int phi, int pitchGain, int Y, float *m, float *b)
{
	//Needs to be checked
	*b = (Y/2) + pitchGain*theta;
	*m = tan(phi*(M_PI/180.0)); //Check whether angles are given in degrees or radians
	return;
}

/* The following function returns 1 if an object is completely above or below the horizon, meaning it should be filtered out. Includes a buffer zone */
int HorizonFilter(int img[240][520], float m, float b, int BUFFER)
{
	//Needs to be checked
	int i, j;
	int xMin = 1000, yMin = 1000, xMax = -1, yMax = -1;
	int y1,y2,y3,y4,x1,x2,x3,x4;
	for(i = 0; i < 240; i++)
	{
		for(j = 0; j < 520; j++)
		{
			if(img[i][j] != 0)
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

int array_sum(int *img, int i, int j, int n_columns, int grid_height, int grid_width)
{
	int k, l;
	int sum = 0;
	for(k=(i*grid_height); k<((i+1)*grid_height); k++)
	{
		for(l=(j*grid_width); l<((j+1)*grid_width); l++)
		{
			sum += img[k*n_columns + l];
		}
	}
	return sum;
}

void grid_counter(int *img, int *p_grid, int n_rows, int n_columns, int grid_height, int grid_width)
{
	//img here is after filtering, so all objects remaining are compiled into one image and need to be avoided
	int i, j;
	
	for(i=0; i<n_rows; i++)
	{
		for(j=0; j<n_columns; j++)
		{
			p_grid[i*n_columns+j] = array_sum(img, i, j, grid_height, grid_width);
		}
	}
	return;
}

int main(void)
{
	int X = 520, Y = 240;
	int GRID_COLUMNS = 7, GRID_ROWS = 9;
	int GRID_WIDTH = (int)(X/GRID_COLUMNS), GRID_HEIGHT = (int)(Y/GRID_ROWS);
	int img[Y*X]; //Convert X by Y array to 1D
	int obj_img[Y*X]; //Possibly temporary
	int grid[GRID_ROWS*GRID_COLUMNS];
	
	printf("Hello");
}
