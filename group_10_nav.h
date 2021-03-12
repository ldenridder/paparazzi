/*
 * @file "modules/group_10_nav/group_10_nav.c"

 */

#ifndef GROUP_10_NAV_H
#define GROUP_10_NAV_H

// settings
extern float max_velocity;               // max flight speed [m/s]
extern float soft_heading_rate;		 // soft heading rate [rad/s]
extern float medium_heading_rate; 	 // medium heading rate [rad/s]
extern float hard_heading_rate;		 // hard heading rate [rad/s]
extern int count;
extern int direction[40][7];

extern void group_10_nav_init(void);
extern void group_10_nav_periodic(void);

#endif

