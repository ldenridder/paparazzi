#ifndef GROUP10AVOIDER_H
#define GROUP10AVOIDER_H

// added
// settings


extern float fast_velocity;               // fast flight speed [m/s]
extern float slow_velocity;  // slow flight speed [m/s]
extern float soft_heading_rate;	 // soft heading rate [rad/s]
extern float hard_heading_rate; 	 // fast heading rate [rad/s]
extern float stop_heading_rate;    // stop heading rate [rad/s]
//extern int floor_count_threshold_low;
//extern int floor_count_threshold_high;
extern int straight_heading_threshold;	 // threshold straight [rad/s]
extern int soft_heading_threshold;	     	 // threshold slow [rad/s]
extern int hard_heading_threshold; 		 // threshold hard [rad/s]
extern int stop_heading_threshold;	 	 // threshold stop [rad/s]



void avoiderInit(void);
void avoiderPeriodic(void);

#endif
