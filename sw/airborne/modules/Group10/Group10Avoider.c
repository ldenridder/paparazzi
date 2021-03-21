#include "modules/project/Group10Avoider.h"

#ifndef PROCESS_FPS
#define PROCESS_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

void avoiderInit(void){
	listener = cv_add_to_device(&VIDEO_CAPTURE_CAMERA, imageProcess, PROCESS_FPS); //Check &COLORFILTER_CAMER!!!!!!!!!!!!!!!
}

void avoiderPeriodic(void){
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
	
	//Navigation
	
}
