#include "modules/Group10/Group10Avoider.h"
#include "subsystems/abi.h"

//added
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include <time.h>

#define GROUP_10_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[group_10->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUP_10_VERBOSE
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
float fast_velocity = 4.f;               // fast flight speed [m/s]
float slow_velocity = 2.f;  		 // slow flight speed (used for turning) [m/s]
float soft_heading_rate = 3.14159f/12;	 // soft heading rate [rad/s]
float hard_heading_rate = 3.14159f/6; 	 // fast heading rate [rad/s]
float stop_heading_rate = 3.14159f/2;    // stop heading rate [rad/s]
int floor_count_threshold_low = 3500;
int floor_count_threshold_high = 3500;
int straight_heading_threshold = 6;	 // threshold straight [rad/s]
int soft_heading_threshold = 4;	     	 // threshold slow [rad/s]
int hard_heading_threshold = 2;		 // threshold hard [rad/s]
int stop_heading_threshold = 5;	 	 // threshold stop [rad/s]



// define and initialise global variables
enum navigation_state_t navigation_state = DIRECTION;   // current state in state machine
float current_velocity = 0.f;		// current velocity [m/s]
float current_heading_rate = 0.f;	// current heading rate [rad/s]
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
int i = 0;
int *navInput;

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

//




static abi_event allowable_distance_ev;
static void allowable_distance_cb(uint8_t __attribute__((unused)) sender_id,
                               int32_t __attribute__((unused)) allowableDistance)
{
	navInput = allowableDistance; //Pointer to a vector containing the allowable distances in each lane
	printf(navInput); //for debugging
}



/*
 * Initialisation function
 */
void avoiderInit(void){
	printf("Got to avoiderInit");
	AbiBindMsgNAVIGATION_VECTOR(NAVIGATION_VECTOR_ID, &allowable_distance_ev, allowable_distance_cb);

//added
	// bind our colorfilter callbacks to receive the color filter outputs
	AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

void avoiderPeriodic(void)
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
 VERBOSE_PRINT("Allowable Distance : %d \n", navInput);
 switch (navigation_state){
    case DIRECTION:
      //count++; // Increase counter for the time of the direction array
      guidance_h_set_guided_heading_rate(current_heading_rate); // Head towards the current heading rate (because in the DIRECTION case otherwise no action would be performed)
      guidance_h_set_guided_body_vel(current_velocity,0); // Keep the speed of the current heading rate (because in the DIRECTION case otherwise no action would be performed)
      if(floor_count < floor_count_threshold_low){ // floor count is compared to the lower threshold
	i = 0;
        navigation_state = FIRST_STOP_FLOOR;
      } else if(navInput[3] > straight_heading_threshold){ //Fly straight when possible
        navigation_state = STRAIGHT;
      } else if(navInput[3] > soft_heading_threshold && navInput[2] > soft_heading_threshold+1 && navInput[2] > navInput[4]){ // Check if an object is in the straight line at some distance whether a soft left or soft right are prefferable
        navigation_state = SOFT_LEFT;
      } else if(navInput[3] > soft_heading_threshold && navInput[4] > soft_heading_threshold+1){
        navigation_state = SOFT_RIGHT;
      } else if(navInput[3] > soft_heading_threshold && navInput[1] > soft_heading_threshold+1 && navInput[1] > navInput[5]){ // Check if an object is in a straight line at some distance and soft left and soft right are also not great, whether hard left or right is better
	navigation_state = HARD_LEFT;
      } else if(navInput[3] > soft_heading_threshold && navInput[5] > soft_heading_threshold+1){
        navigation_state = HARD_RIGHT;
      } else if(navInput[3] > soft_heading_threshold){ //Fly slower straight
        navigation_state = SLOW_STRAIGHT;
      } else if(navInput[3] > hard_heading_threshold && navInput[1] > hard_heading_threshold+1 && navInput[1] > navInput[5]){ // Check if an object is closer by whetehr hard left or hard right is a good way to fly towards
	navigation_state = HARD_LEFT;
      } else if(navInput[3] > hard_heading_threshold && navInput[5] > hard_heading_threshold+1){
        navigation_state = HARD_RIGHT;
      } else if(navInput[0] > stop_heading_threshold && navInput[0] > navInput[6]){ // If there still is no good option to fly towards, start rotating to either left or right, whichever are the best
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
	  if (floor_count > floor_count_threshold_high && navInput[3] > straight_heading_threshold) {
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

