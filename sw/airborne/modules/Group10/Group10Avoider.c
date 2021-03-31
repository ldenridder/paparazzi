#include "modules/Group10/Group10Avoider.h"
#include "subsystems/abi.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include <time.h>

#define GROUP_10_VERBOSE TRUE


/*
For the navigation it was chosen to make different cases with the default case being direction. This case loops through a decision tree based on the floor count and input from th direction array
*/

enum navigation_state_t {
  DIRECTION,
  STOP_LEFT_FLOOR,
  STOP_RIGHT_FLOOR,
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
float fast_velocity = 2.f;              // fast flight speed [m/s]
float slow_velocity = 1.f;  		 	// slow flight speed (used for turning) [m/s]
float soft_heading_rate = 3.14159f/12;	// soft heading rate [rad/s]
float hard_heading_rate = 3.14159f/6; 	// fast heading rate [rad/s]
float stop_heading_rate = 3.14159f/2;   // stop heading rate [rad/s]
int floor_count_threshold = 750;	 	// threshold for the green pixel count
int straight_heading_threshold = 3;	    // threshold straight
int soft_heading_threshold = 2;	     	// threshold slow
int hard_heading_threshold = 1;		    // threshold hard
int stop_heading_threshold = 0;	 	// threshold stop



// define and initialise global variables
enum navigation_state_t navigation_state = DIRECTION;   // current state in state machine
float current_velocity = 0.f;							// current velocity [m/s]
float current_heading_rate = 0.f;						// current heading rate [rad/s]
int32_t floor_count = 0;             					// green color count from color filter for floor detection
int32_t floor_centroid = 0;          					// floor detector centroid in y direction (along the horizon)
int i = 0;												// counter of the green detection
int navInput1 = 0;										// part of the allowable distance area
int navInput2 = 0;										// part of the allowable distance area
int navInput3 = 0;										// part of the allowable distance area
int navInput4 = 0;										// part of the allowable distance area
int navInput5 = 0;										// part of the allowable distance area
int navInput6 = 0;										// part of the allowable distance area
int navInput7 = 0;										// part of the allowable distance area
int green_1 = 0;										// part of the green counter
int green_2 = 0;										// part of the green counter
int green_3 = 0;										// part of the green counter
int green_4 = 0;										// part of the green counter


#ifndef NAVIGATION_VECTOR_ID
#endif



static abi_event allowable_distance_ev;
static void allowable_distance_cb(uint8_t __attribute__((unused)) sender_id,
                               int allowableDistance1,
							   int allowableDistance2,
							   int allowableDistance3,
							   int allowableDistance4,
							   int allowableDistance5,
							   int allowableDistance6,
							   int allowableDistance7,
							   int green1,
							   int green2,
							   int green3,
							   int green4)
{
	navInput1 = allowableDistance1;
	navInput2 = allowableDistance2;
	navInput3 = allowableDistance3;
	navInput4 = allowableDistance4;
	navInput5 = allowableDistance5;
	navInput6 = allowableDistance6;
	navInput7 = allowableDistance7;
	green_1 = green1;
	green_2 = green2;
	green_3 = green3;
	green_4 = green4;
}



/*
 * Initialisation function
 */
void avoiderInit(void){
	AbiBindMsgNAVIGATION_VECTOR(NAVIGATION_VECTOR_ID, &allowable_distance_ev, allowable_distance_cb);
}

void avoiderPeriodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = DIRECTION; // Direction is the decision tree where the decisions are made
    return;
  }

 printf("Current velocity: %f \n", current_velocity);
 printf("Current heading rate: %f \n", current_heading_rate);
 printf("Navigation state: %d \n", navigation_state);
 switch (navigation_state){
    case DIRECTION:
      guidance_h_set_guided_heading_rate(current_heading_rate); // Head towards the current heading rate (because in the DIRECTION case otherwise no action would be performed)
      guidance_h_set_guided_body_vel(current_velocity,0); // Keep the speed of the current heading rate (because in the DIRECTION case otherwise no action would be performed)
      if(green_1 <floor_count_threshold && green_1 < green_2){ // floor count is compared to the lower threshold
        i = 0;
    	navigation_state = STOP_LEFT_FLOOR;
      } else if(green_2 <floor_count_threshold){ // floor count is compared to the lower threshold
        i = 0;
    	navigation_state = STOP_RIGHT_FLOOR;
      } else if(navInput4 > straight_heading_threshold){ //Fly straight when possible
        navigation_state = STRAIGHT;
      } else if(navInput4 > soft_heading_threshold && navInput3 > soft_heading_threshold+1 && navInput3 > navInput5){ // Check if an object is in the straight line at some distance whether a soft left or soft right are prefferable
        navigation_state = SOFT_LEFT;
      } else if(navInput4 > soft_heading_threshold && navInput5 > soft_heading_threshold+1){
        navigation_state = SOFT_RIGHT;
      } else if(navInput4 > soft_heading_threshold && navInput2 > soft_heading_threshold+1 && navInput2 > navInput5){ // Check if an object is in a straight line at some distance and soft left and soft right are also not great, whether hard left or right is better
     	navigation_state = HARD_LEFT;
      } else if(navInput4 > soft_heading_threshold && navInput5 > soft_heading_threshold+1){
        navigation_state = HARD_RIGHT;
      } else if(navInput4 > soft_heading_threshold){ //Fly slower straight
        navigation_state = SLOW_STRAIGHT;
      } else if(navInput4 > hard_heading_threshold && navInput2 > hard_heading_threshold+1 && navInput2 > navInput5){ // Check if an object is closer by whetehr hard left or hard right is a good way to fly towards
     	navigation_state = HARD_LEFT;
      } else if(navInput4 > hard_heading_threshold && navInput5 > hard_heading_threshold+1){
        navigation_state = HARD_RIGHT;
      } else if(navInput1 > stop_heading_threshold && navInput1 > navInput7){ // If there still is no good option to fly towards, start rotating to either left or right, whichever are the best
	    navigation_state = STOP_LEFT;
      } else{
        navigation_state = STOP_RIGHT;
      }
      break;



    case STOP_LEFT_FLOOR: //If there is to little floor on the right, the drone will rotate with 90 degrees towards the left
	  guidance_h_set_guided_body_vel(0, 0);


	  current_velocity = 0;
	  if(i == 3){ // After three iterations the 90 degrees rotations is achieved
		guidance_h_set_guided_heading_rate(0);
		current_heading_rate = 0; //The heading rate is set to 0, so the drone is completely still
		navigation_state = DIRECTION;
	  } else{
		guidance_h_set_guided_heading_rate(-stop_heading_rate);
		current_heading_rate = -stop_heading_rate;
		navigation_state = STOP_LEFT_FLOOR;
		i++;
	  }
	  break;

    case STOP_RIGHT_FLOOR: //If there is to little floor on the left, the drone will rotate with 90 degrees towards the right
	  guidance_h_set_guided_body_vel(0, 0);

	  current_velocity = 0;
	  if(i == 3){ // After three iterations the 90 degrees rotations is achieved
		guidance_h_set_guided_heading_rate(0);
		current_heading_rate = 0; //The heading rate is set to 0, so the drone is completely still
		navigation_state = DIRECTION;
	  } else{
		guidance_h_set_guided_heading_rate(stop_heading_rate);
		current_heading_rate = stop_heading_rate;
		navigation_state = STOP_RIGHT_FLOOR;
		i++;
	  }
	  break;

    case STOP_LEFT: // The drone is stopped and rotates with a large heading rate to the left
      guidance_h_set_guided_body_vel(0, 0);
      guidance_h_set_guided_heading_rate(-stop_heading_rate);

      current_heading_rate = -stop_heading_rate;
      current_velocity = 0;
      navigation_state = DIRECTION;

      break;

    case HARD_LEFT: // The drone flies slow and rotates with a pretty large heading rate to the left
      guidance_h_set_guided_body_vel(slow_velocity, 0);
      guidance_h_set_guided_heading_rate(-hard_heading_rate);

      current_heading_rate = -hard_heading_rate;
      current_velocity = slow_velocity;
      navigation_state = DIRECTION;

      break;

    case SOFT_LEFT: // The drone flies fast and rotates with a small heading rate to the left
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

    case SOFT_RIGHT: // The drone flies fast and rotates with a small heading rate to the right
      guidance_h_set_guided_heading_rate(soft_heading_rate);
      guidance_h_set_guided_body_vel(fast_velocity, 0);

      current_heading_rate = soft_heading_rate;
      current_velocity = fast_velocity;
      navigation_state = DIRECTION;
      break;

    case HARD_RIGHT: // The drone flies slow and rotates with a pretty large heading rate to the right
      guidance_h_set_guided_heading_rate(hard_heading_rate);
      guidance_h_set_guided_body_vel(slow_velocity, 0);

      current_heading_rate = hard_heading_rate;
      current_velocity = slow_velocity;
      navigation_state = DIRECTION;
      break;

    case STOP_RIGHT:  // The drone is stopped and rotates with a large heading rate to the right
      guidance_h_set_guided_heading_rate(stop_heading_rate);
      guidance_h_set_guided_body_vel(0, 0);

      current_heading_rate = stop_heading_rate;
      current_velocity = 0;
      navigation_state = DIRECTION;
      break;

    case SLOW_STRAIGHT: // The drone flies straight but slower because of obstacles nearby
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

