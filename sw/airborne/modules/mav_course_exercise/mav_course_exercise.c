/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/MAV_COURSE_EXERCISE/MAV_COURSE_EXERCISE_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the MAV_COURSE_EXERCISE_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler MAV_COURSE_EXERCISE.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/mav_course_exercise/mav_course_exercise.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

#define MAV_COURSE_EXERCISE_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[mav_course_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if MAV_COURSE_EXERCISE_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


enum navigation_state_t {
  SAFE,
  CAUTIOUS,
  OBSTACLE_FOUND,
  CLOSE_OBSTACLE_FOUND,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_color_count_frac = 0.2f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.005f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 35.f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(25.f);  // heading change setpoint for avoidance [rad/s]
float avoidance_heading_direction = 13.f;  // heading change direction for avoidance [rad/s]
float last_avoidance_heading_direction = 13.f;  // heading change direction for avoidance [rad/s]
float slow_avoidance_heading_direction = 5.f;  // heading change direction for avoidance [rad/s]
float last_slow_avoidance_heading_direction = 5.f;
float rapid_avoidance_heading_direction = 25.f;  // heading change direction for avoidance [rad/s]


// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t last_color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
int16_t obstacle_free_confidence = 2;   // a measure of how certain we are that the way ahead if safe.

const int16_t max_trajectory_confidence = 10;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef MAV_COURSE_EXERCISE_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define MAV_COURSE_EXERCISE_VISUAL_DETECTION_ID to the orange filter
#error Please define MAV_COURSE_EXERCISE_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

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
void mav_course_exercise_init(void)
{
  // Initialise random values
  srand(time(NULL));

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(MAV_COURSE_EXERCISE_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void mav_course_exercise_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    obstacle_free_confidence = 2;
    navigation_state = SAFE;
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 4;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed_sp = fminf(oag_max_speed, 3.5f * obstacle_free_confidence);

  VERBOSE_PRINT("SPEED SP: %f\n", speed_sp);
  VERBOSE_PRINT(" OBSTACLE FREE COUNT: %d \n", obstacle_free_confidence);
 switch (navigation_state){
    case SAFE:
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.22){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence >= 4){
        guidance_h_set_guided_body_vel(speed_sp, 0);
        guidance_h_set_guided_heading_rate(0);
      } else if (obstacle_free_confidence > 2){
	navigation_state = CAUTIOUS;
      } else if (obstacle_free_confidence > 0){
	navigation_state = OBSTACLE_FOUND;
      } else {
        navigation_state = CLOSE_OBSTACLE_FOUND;
      }
      break;

    case CAUTIOUS:
       // don't stop
       guidance_h_set_guided_body_vel(speed_sp, 0);
       guidance_h_set_guided_heading_rate(0);
       if (color_count > last_color_count) {
         slow_avoidance_heading_direction = -last_slow_avoidance_heading_direction;
       } else {
         slow_avoidance_heading_direction = last_slow_avoidance_heading_direction;
       }
       
       guidance_h_set_guided_heading_rate(slow_avoidance_heading_direction * oag_heading_rate);
       VERBOSE_PRINT("Set avoidance increment to: %f\n", slow_avoidance_heading_direction * oag_heading_rate);
       
       navigation_state = SAFE;
       break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // select new search direction
      if (color_count > last_color_count) {
         avoidance_heading_direction = -last_avoidance_heading_direction;
       } else {
         avoidance_heading_direction = last_avoidance_heading_direction;
       }

      guidance_h_set_guided_heading_rate(avoidance_heading_direction * oag_heading_rate);
      VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);

      navigation_state = SAFE;
      break;

    case CLOSE_OBSTACLE_FOUND:
      // stop
      guidance_h_set_guided_body_vel(0, 0);
      
      guidance_h_set_guided_heading_rate(rapid_avoidance_heading_direction * oag_heading_rate);
      VERBOSE_PRINT("Set avoidance increment to: %f\n", rapid_avoidance_heading_direction * oag_heading_rate);
      navigation_state = SAFE;
      break;

    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_guided_heading_rate(rapid_avoidance_heading_direction * RadOfDeg(10));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && -rapid_avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      obstacle_free_confidence++;
      break;
    default:
      break;
  }
  return;

last_color_count = color_count;

}




