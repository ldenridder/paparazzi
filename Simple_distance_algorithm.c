/*
 * Simple_distance_algorithm.c
 *
 *  Created on: Mar 18, 2021
 *      Author: vi-hung
 */

#include <stdio.h>
#include <math.h>

/*function declaration*/
double vel_function(double, double);
double distance_function(double,double,double,double);



int main(int argc, char *argv[]) {
	//double vel = vel_function(5.0,5.0);
		double dist = distance_function(8,8,0.1,5);

		printf("Distance to object is %lf\n",dist);
	}


double vel_function(double vx, double vy){
	double d = vx*vx + vy*vy;
	return sqrt(d);
}

double distance_function(double obj_width1, double obj_width2, double time_lapse, double velocity){

  double dis_trav = velocity*time_lapse;
  double d;
  double obj_dist;

  if (obj_width1 > obj_width2){
     d = dis_trav + dis_trav/(obj_width1/obj_width2 -1);
     obj_dist = d;
  }
  else if (obj_width1 < obj_width2){
      d = dis_trav/(1.0 - obj_width1/obj_width2);
      obj_dist = d-dis_trav;
  }
  else{
      obj_dist = NAN;
  }

  return obj_dist;
}


