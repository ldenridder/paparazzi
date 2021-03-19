/*
 * @file "modules/group_10_nav/group_10_nav.c"

 */

#ifndef GROUP_10_NAV_H
#define GROUP_10_NAV_H

// settings
extern float fast_velocity;               // fast flight speed [m/s]
extern float slow_velocity;  // slow flight speed [m/s]
extern float soft_heading_rate;	 // soft heading rate [rad/s]
extern float hard_heading_rate; 	 // fast heading rate [rad/s]
extern float stop_heading_rate;    // stop heading rate [rad/s]
extern int edge_line;
extern int straight_heading_threshold;	 // threshold straight [rad/s]
extern int soft_heading_threshold;	     	 // threshold slow [rad/s]
extern int hard_heading_threshold; 		 // threshold hard [rad/s]
extern int stop_heading_threshold;	 	 // threshold stop [rad/s]
extern int count;
extern int direction[40][7];

extern void group_10_nav_init(void);
extern void group_10_nav_periodic(void);

#endif

