
#include "Marlin.h"
#include "stepper.h"
#include "planner.h"



void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1,
  uint8_t axis_linear, float feed_rate, float radius, uint8_t isclockwise, uint8_t extruder)
{


  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float linear_travel = target[axis_linear] - position[axis_linear];
  float extruder_travel = target[E_AXIS] - position[E_AXIS];
  float r_axis0 = -offset[axis_0];
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;


  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (angular_travel < 0) { angular_travel += 2*M_PI; }
  if (isclockwise) { angular_travel -= 2*M_PI; }

  float millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel < 0.001) { return; }
  uint16_t segments = floor(millimeters_of_travel/MM_PER_ARC_SEGMENT);
  if(segments == 0) segments = 1;



  float theta_per_segment = angular_travel/segments;
  float linear_per_segment = linear_travel/segments;
  float extruder_per_segment = extruder_travel/segments;


  
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment;
  float sin_T = theta_per_segment;

  float arc_target[4];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;


  arc_target[axis_linear] = position[axis_linear];


  arc_target[E_AXIS] = position[E_AXIS];

  for (i = 1; i<segments; i++) {

    if (count < N_ARC_CORRECTION) {

      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {


      cos_Ti = cos(i*theta_per_segment);
      sin_Ti = sin(i*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }


    arc_target[axis_0] = center_axis0 + r_axis0;
    arc_target[axis_1] = center_axis1 + r_axis1;
    arc_target[axis_linear] += linear_per_segment;
    arc_target[E_AXIS] += extruder_per_segment;

    clamp_to_software_endstops(arc_target);
    plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], arc_target[E_AXIS], feed_rate, extruder);

  }

  plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feed_rate, extruder);


}
