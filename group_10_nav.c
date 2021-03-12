
 /* @file "modules/GROUP_10_NAV/GROUP_10_NAV.c"

 */

#include "modules/group_10_nav/group_10_nav.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

#define GROUP_10_NAV_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[group_10_nav->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUP_10_NAV_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


enum navigation_state_t {
  DIRECTION,
  STOP_LEFT,
  HARD_LEFT,
  SOFT_LEFT,
  STRAIGHT,
  SOFT_RIGHT,
  HARD_RIGHT,
  STOP_RIGHT
};

// define settings
float max_velocity = 15.f;               // max flight speed [m/s]
float soft_heading_rate = 0.4f;		 // soft heading rate [rad/s]
float medium_heading_rate = 0.8f; 	 // medium heading rate [rad/s]
float hard_heading_rate = 1.6f;		 // hard heading rate [rad/s]
int count = -1;				 // counter for the direction
int direction[40][7] = {0, 0, 0, 9, 0, 0, 0, // (1 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (2
			0, 0, 9, 5, 0, 0, 0, // (3 SOFT LEFT
			0, 0, 9, 5, 0, 0, 0, // (4
			0, 0, 0, 9, 0, 0, 0, // (5 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (6
			0, 0, 0, 9, 0, 0, 0, // (7
			0, 0, 0, 9, 0, 0, 0, // (8
			9, 0, 0, 2, 0, 0, 0, // (9 STOP LEFT
			9, 0, 0, 2, 0, 0, 0, // (10
			9, 0, 0, 2, 0, 0, 0, // (11
			9, 0, 0, 2, 0, 0, 0, // (12
			0, 0, 0, 9, 0, 0, 0, // (13 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (14
			0, 0, 0, 9, 0, 0, 0, // (15
			0, 0, 0, 9, 0, 0, 0, // (16
			0, 0, 0, 3, 0, 6, 0, // (17 HARD RIGHT 
			0, 0, 0, 3, 0, 6, 0, // (18
			0, 0, 0, 3, 0, 6, 0, // (19
			0, 0, 0, 3, 0, 6, 0, // (20
			0, 0, 0, 9, 0, 0, 0, // (21 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (22
			0, 0, 0, 9, 0, 0, 0, // (23
			0, 0, 0, 9, 0, 0, 0, // (24
			0, 6, 0, 3, 0, 0, 0, // (25 HARD LEFT
			0, 6, 0, 3, 0, 0, 0, // (26
			0, 6, 0, 3, 0, 0, 0, // (27
			0, 6, 0, 3, 0, 0, 0, // (28
			0, 6, 0, 3, 0, 0, 0, // (29
			0, 6, 0, 3, 0, 0, 0, // (30
			0, 6, 0, 3, 0, 0, 0, // (31
			0, 6, 0, 3, 0, 0, 0, // (32
			0, 0, 0, 9, 0, 0, 0, // (33 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (34 
			0, 0, 0, 9, 0, 0, 0, // (35
			0, 0, 0, 9, 0, 0, 0, // (36
			0, 0, 0, 0, 0, 0, 8, // (37 STOP RIGHT
			0, 0, 0, 0, 0, 0, 8, // (38
			0, 0, 0, 0, 0, 0, 8, // (39
			0, 0, 0, 0, 0, 0, 8};   // direction confidence  


// define and initialise global variables
enum navigation_state_t navigation_state = DIRECTION;   // current state in state machine
float current_velocity = 0.f;		// current velocity [m/s]
float current_heading_rate = 0.f;	// current heading rate [rad/s]
  


// This call back will be used to receive the color count from the orange detector


/*
 * Initialisation function
 */
void group_10_nav_init(void)
{
  // Initialise random values
  srand(time(NULL));

  // bind our colorfilter callbacks to receive the color filter outputs

}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void group_10_nav_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = DIRECTION;
    return;
  }

  // compute current color thresholds
 
 VERBOSE_PRINT("Current velocity: %f \n", current_velocity);
 VERBOSE_PRINT("Current heading rate: %f \n", current_heading_rate);
 VERBOSE_PRINT("Navigation state: %d \n", navigation_state);
 VERBOSE_PRINT("Time state: %d \n", count); 
 switch (navigation_state){
    case DIRECTION:
      count++;
      guidance_h_set_guided_heading_rate(current_heading_rate);
      guidance_h_set_guided_body_vel(current_velocity,0);
      if(direction[count][3] > 6){
        navigation_state = STRAIGHT;
      } else if(direction[count][3] > 4 && direction[count][2] > 5 && direction[count][2] > direction[count][4]){
        navigation_state = SOFT_LEFT;
      } else if(direction[count][3] > 4 && direction[count][4] > 5){
        navigation_state = SOFT_RIGHT;
      } else if(direction[count][3] > 4 && direction[count][1] > 5 && direction[count][1] > direction[count][5]){
	navigation_state = HARD_LEFT;
      } else if(direction[count][3] > 4 && direction[count][5] > 5){
        navigation_state = HARD_RIGHT;
      } else if(direction[count][3] > 2 && direction[count][1] > 3 && direction[count][1] > direction[count][5]){
	navigation_state = HARD_LEFT;
      } else if(direction[count][3] > 2 && direction[count][5] > 3){
        navigation_state = HARD_RIGHT;
      } else if(direction[count][0] > 5 && direction[count][0] > direction[count][6]){
	navigation_state = STOP_LEFT;
      } else{
        navigation_state = STOP_RIGHT;
      }
      break;


    case STOP_LEFT:
      guidance_h_set_guided_body_vel(0, 0);
      guidance_h_set_guided_heading_rate(-hard_heading_rate);

      current_heading_rate = -hard_heading_rate;
      current_velocity = 0;
      navigation_state = DIRECTION;

      break;

    case HARD_LEFT:
      guidance_h_set_guided_body_vel(0.5*max_velocity, 0);
      guidance_h_set_guided_heading_rate(-medium_heading_rate);

      current_heading_rate = -medium_heading_rate;
      current_velocity = 0.5*max_velocity;
      navigation_state = DIRECTION;

      break;

    case SOFT_LEFT:
      guidance_h_set_guided_heading_rate(-soft_heading_rate);
      guidance_h_set_guided_body_vel(max_velocity, 0);

      current_heading_rate = -soft_heading_rate;
      current_velocity = max_velocity;
      navigation_state = DIRECTION;

      break;

    case STRAIGHT:
      guidance_h_set_guided_heading_rate(0);
      guidance_h_set_guided_body_vel(max_velocity, 0);

      current_heading_rate = 0;
      current_velocity = max_velocity;
      navigation_state = DIRECTION;
      break;

    case SOFT_RIGHT:
      guidance_h_set_guided_heading_rate(soft_heading_rate);
      guidance_h_set_guided_body_vel(max_velocity, 0);

      current_heading_rate = soft_heading_rate;
      current_velocity = max_velocity;
      navigation_state = DIRECTION;
      break;

    case HARD_RIGHT:
      guidance_h_set_guided_heading_rate(medium_heading_rate);
      guidance_h_set_guided_body_vel(0.5*max_velocity, 0);

      current_heading_rate = medium_heading_rate;
      current_velocity = 0.5*max_velocity;
      navigation_state = DIRECTION;
      break;

    case STOP_RIGHT:
      guidance_h_set_guided_heading_rate(hard_heading_rate);
      guidance_h_set_guided_body_vel(0, 0);

      current_heading_rate = hard_heading_rate;
      current_velocity = 0;
      navigation_state = DIRECTION;
      break;

    default:
      break;
  }
  return;

}




