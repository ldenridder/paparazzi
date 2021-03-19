
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

/*
For the navigation it was chosen to make different cases with the default case being direction. This case loops through a decision tree based on the floor count and input from th direction array 
*/

enum navigation_state_t {
  DIRECTION,
  FIRST_STOP_FLOOR,
  SECOND_STOP_FLOOR,
  STOP_LEFT,
  HARD_LEFT,
  SOFT_LEFT,
  STRAIGHT,
  SLOW_STRAIGHT,
  SOFT_RIGHT,
  HARD_RIGHT,
  STOP_RIGHT
};

// define settings
float fast_velocity = 5.f;               // fast flight speed [m/s]
float slow_velocity = 2.5f;  		 // slow flight speed (used for turning) [m/s]
float soft_heading_rate = 3.14159f/8;	 // soft heading rate [rad/s]
float hard_heading_rate = 3.14159f/4; 	 // fast heading rate [rad/s]
float stop_heading_rate = 3.14159f/2;    // stop heading rate [rad/s]
int floor_count_threshold_low = 3500;
int floor_count_threshold_high = 3500;
int straight_heading_threshold = 6;	 // threshold straight [rad/s]
int soft_heading_threshold = 4;	     	 // threshold slow [rad/s]
int hard_heading_threshold = 2;		 // threshold hard [rad/s]
int stop_heading_threshold = 5;	 	 // threshold stop [rad/s]



int count = -1;				 // counter for the direction
int direction[52][7] = {0, 0, 0, 9, 0, 0, 0, // (1 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (2
			0, 0, 9, 5, 0, 0, 0, // (3 SOFT LEFT
			0, 0, 9, 5, 0, 0, 0, // (4
			0, 0, 0, 9, 0, 0, 0, // (5 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (6
			0, 0, 0, 9, 0, 0, 0, // (7
			0, 0, 0, 9, 0, 0, 0, // (8
//			9, 0, 0, 2, 0, 0, 0, // (9 STOP LEFT
//			9, 0, 0, 2, 0, 0, 0, // (10
//			9, 0, 0, 2, 0, 0, 0, // (11
//			9, 0, 0, 2, 0, 0, 0, // (12
			0, 0, 0, 9, 0, 0, 0, // (13 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (14
			0, 0, 0, 9, 0, 0, 0, // (15
			0, 0, 0, 9, 0, 0, 0, // (16
			0, 0, 0, 9, 0, 0, 0, // (13 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (14
			0, 0, 0, 9, 0, 0, 0, // (15
			0, 0, 0, 9, 0, 0, 0, // (16
			0, 0, 0, 9, 0, 0, 0, // (13 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (14
			0, 0, 0, 9, 0, 0, 0, // (15
			0, 0, 0, 9, 0, 0, 0, // (16
			0, 0, 0, 9, 0, 0, 0, // (13 STRAIGHT
			0, 0, 0, 9, 0, 0, 0, // (14
			0, 0, 0, 9, 0, 0, 0, // (15
			0, 0, 0, 9, 0, 0, 0, // (16s
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
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon) 
int i = 0;

// This call back will be used to receive the color count from the orange detector




#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}



/*
 * Initialisation function
 */
void group_10_nav_init(void)
{
  // Initialise random values
  srand(time(NULL));

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
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
 VERBOSE_PRINT("Color_count: %d  threshold: %d: \n", floor_count, floor_count_threshold_low, floor_count_threshold_high);
 VERBOSE_PRINT("Current velocity: %f \n", current_velocity);
 VERBOSE_PRINT("Current heading rate: %f \n", current_heading_rate);
 VERBOSE_PRINT("Navigation state: %d \n", navigation_state);
 VERBOSE_PRINT("Time state: %d \n", count); 
 switch (navigation_state){
    case DIRECTION:
      count++; // Increase counter for the time of the direction array
      guidance_h_set_guided_heading_rate(current_heading_rate); // Head towards the current heading rate (because in the DIRECTION case otherwise no action would be performed)
      guidance_h_set_guided_body_vel(current_velocity,0); // Keep the speed of the current heading rate (because in the DIRECTION case otherwise no action would be performed)
      if(floor_count < floor_count_threshold_low){ // floor count is compared to the lower threshold
	i = 0;
        navigation_state = FIRST_STOP_FLOOR;
      } else if(direction[count][3] > straight_heading_threshold){ //Fly straight when possible
        navigation_state = STRAIGHT;
      } else if(direction[count][3] > soft_heading_threshold && direction[count][2] > soft_heading_threshold+1 && direction[count][2] > direction[count][4]){ // Check if an object is in the straight line at some distance whether a soft left or soft right are prefferable
        navigation_state = SOFT_LEFT;
      } else if(direction[count][3] > soft_heading_threshold && direction[count][4] > soft_heading_threshold+1){ 
        navigation_state = SOFT_RIGHT;
      } else if(direction[count][3] > soft_heading_threshold && direction[count][1] > soft_heading_threshold+1 && direction[count][1] > direction[count][5]){ // Check if an object is in a straight line at some distance and soft left and soft right are also not great, whether hard left or right is better
	navigation_state = HARD_LEFT;
      } else if(direction[count][3] > soft_heading_threshold && direction[count][5] > soft_heading_threshold+1){
        navigation_state = HARD_RIGHT;
      } else if(direction[count][3] > soft_heading_threshold){ //Fly slower straight
        navigation_state = SLOW_STRAIGHT;
      } else if(direction[count][3] > hard_heading_threshold && direction[count][1] > hard_heading_threshold+1 && direction[count][1] > direction[count][5]){ // Check if an object is closer by whetehr hard left or hard right is a good way to fly towards
	navigation_state = HARD_LEFT;
      } else if(direction[count][3] > hard_heading_threshold && direction[count][5] > hard_heading_threshold+1){
        navigation_state = HARD_RIGHT;
      } else if(direction[count][0] > stop_heading_threshold && direction[count][0] > direction[count][6]){ // If there still is no good option to fly towards, start rotating to either left or right, whichever are the best
	navigation_state = STOP_LEFT;
      } else{
        navigation_state = STOP_RIGHT;
      }
      break;




   
    case FIRST_STOP_FLOOR: //If there is to little floor, the drone will rotate with 90 degrees towards the left
      guidance_h_set_guided_body_vel(0, 0);
      guidance_h_set_guided_heading_rate(-stop_heading_rate);
      
      current_heading_rate = -stop_heading_rate;
      current_velocity = 0;
      if(i == 3){ // After three iterations the 90 degrees rotations is achieved
        navigation_state = SECOND_STOP_FLOOR;
      } else{
        navigation_state = FIRST_STOP_FLOOR;
        i++;
      }
      
      break;

    case SECOND_STOP_FLOOR: //This checks whether this time there is enough floor and there are no objects in front of you otherwise it rotates another 90 degrees
      guidance_h_set_guided_body_vel(0, 0);
      if (floor_count > floor_count_threshold_high && direction[count][3] > straight_heading_threshold) {
        guidance_h_set_guided_heading_rate(0);
        current_heading_rate = 0;
        current_velocity = 0;
        navigation_state = DIRECTION;
      }
      else {
        guidance_h_set_guided_heading_rate(-stop_heading_rate);
        current_heading_rate = 0;
        current_velocity = 0;
        if(i == 6){
          navigation_state = DIRECTION;
        } else{
          navigation_state = SECOND_STOP_FLOOR;
          i++;
      }
      }
      
      break;

    case STOP_LEFT: // The drone is stopped and rotates with a large heading rate
      guidance_h_set_guided_body_vel(0, 0);
      guidance_h_set_guided_heading_rate(-stop_heading_rate);

      current_heading_rate = -stop_heading_rate;
      current_velocity = 0;
      navigation_state = DIRECTION;

      break;

    case HARD_LEFT: // The drone flies slow and rotates with a pretty large heading rate
      guidance_h_set_guided_body_vel(slow_velocity, 0);
      guidance_h_set_guided_heading_rate(-hard_heading_rate);

      current_heading_rate = -hard_heading_rate;
      current_velocity = slow_velocity;
      navigation_state = DIRECTION;

      break;

    case SOFT_LEFT: // The drone flies fast and rotates with a small heading rate
      guidance_h_set_guided_heading_rate(-soft_heading_rate);
      guidance_h_set_guided_body_vel(fast_velocity, 0);

      current_heading_rate = -soft_heading_rate;
      current_velocity = fast_velocity;
      navigation_state = DIRECTION;

      break;

    case STRAIGHT: // The drone flies straight
      guidance_h_set_guided_heading_rate(0);
      guidance_h_set_guided_body_vel(fast_velocity, 0);

      current_heading_rate = 0;
      current_velocity = fast_velocity;
      navigation_state = DIRECTION;
      break;

    case SOFT_RIGHT:
      guidance_h_set_guided_heading_rate(soft_heading_rate);
      guidance_h_set_guided_body_vel(fast_velocity, 0);

      current_heading_rate = soft_heading_rate;
      current_velocity = fast_velocity;
      navigation_state = DIRECTION;
      break;

    case HARD_RIGHT:
      guidance_h_set_guided_heading_rate(hard_heading_rate);
      guidance_h_set_guided_body_vel(slow_velocity, 0);

      current_heading_rate = hard_heading_rate;
      current_velocity = slow_velocity;
      navigation_state = DIRECTION;
      break;

    case STOP_RIGHT:
      guidance_h_set_guided_heading_rate(stop_heading_rate);
      guidance_h_set_guided_body_vel(0, 0);

      current_heading_rate = stop_heading_rate;
      current_velocity = 0;
      navigation_state = DIRECTION;
      break;

    case SOFT_STRAIGHT: // The drone flies straight but slower because of obstacles nearby
      guidance_h_set_guided_heading_rate(0);
      guidance_h_set_guided_body_vel(slow_velocity, 0);

      current_heading_rate = 0;
      current_velocity = slow_velocity;
      navigation_state = DIRECTION;
      break;
    default:
      break;
  }
  return;

}




