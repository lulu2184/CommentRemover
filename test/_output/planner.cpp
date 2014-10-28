





#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"





unsigned long minsegmenttime;
float max_feedrate[4]; 
float axis_steps_per_unit[4];
unsigned long max_acceleration_units_per_sq_second[4]; 
float minimumfeedrate;
float acceleration;         
float retract_acceleration; 
float max_xy_jerk; 
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];


long position[4];   
static float previous_speed[4]; 
static float previous_nominal_speed; 

#ifdef AUTOTEMP
float autotemp_max=250;
float autotemp_min=210;
float autotemp_factor=0.1;
bool autotemp_enabled=false;
#endif




block_t block_buffer[BLOCK_BUFFER_SIZE];            
volatile unsigned char block_buffer_head;           
volatile unsigned char block_buffer_tail;           




#ifdef PREVENT_DANGEROUS_EXTRUDE
float extrude_min_temp=EXTRUDE_MINTEMP;
#endif
#ifdef XY_FREQUENCY_LIMIT
#define MAX_FREQ_TIME (1000000.0/XY_FREQUENCY_LIMIT)

static unsigned char old_direction_bits = 0;               
static long x_segment_time[3]={MAX_FREQ_TIME + 1,0,0};     
static long y_segment_time[3]={MAX_FREQ_TIME + 1,0,0};
#endif



static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { 
    block_index = 0; 
  }
  return(block_index);
}



static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) { 
    block_index = BLOCK_BUFFER_SIZE; 
  }
  block_index--;
  return(block_index);
}







FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration!=0) {
    return((target_rate*target_rate-initial_rate*initial_rate)/
      (2.0*acceleration));
  }
  else {
    return 0.0;  
  }
}






FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
  if (acceleration!=0) {
    return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
      (4.0*acceleration) );
  }
  else {
    return 0.0;  
  }
}



void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); 
  unsigned long final_rate = ceil(block->nominal_rate*exit_factor); 

  
  if(initial_rate <120) {
    initial_rate=120; 
  }
  if(final_rate < 120) {
    final_rate=120;  
  }

  long acceleration = block->acceleration_st;
  int32_t accelerate_steps =
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration));

  
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;

  
  
  
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps,0); 
    accelerate_steps = min((uint32_t)accelerate_steps,block->step_event_count);
    plateau_steps = 0;
  }

#ifdef ADVANCE
  volatile long initial_advance = block->advance*entry_factor*entry_factor; 
  volatile long final_advance = block->advance*exit_factor*exit_factor;
#endif 

  
  
  CRITICAL_SECTION_START;  
  if(block->busy == false) { 
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps+plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
#ifdef ADVANCE
    block->initial_advance = initial_advance;
    block->final_advance = final_advance;
#endif 
  }
  CRITICAL_SECTION_END;
}                    



FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}











void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { 
    return; 
  }

  if (next) {
    
    
    
    if (current->entry_speed != current->max_entry_speed) {

      
      
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed,
        max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } 
      else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } 
}



void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;
  
  
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END
  
  if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = { 
      NULL, NULL, NULL         };
    while(block_index != tail) { 
      block_index = prev_block_index(block_index); 
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}


void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!previous) { 
    return; 
  }

  
  
  
  
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed,
      max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

      
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}



void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = { 
    NULL, NULL, NULL   };

  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}




void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      
      if (current->recalculate_flag || next->recalculate_flag) {
        
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
        next->entry_speed/current->nominal_speed);
        current->recalculate_flag = false; 
      }
    }
    block_index = next_block_index( block_index );
  }
  
  if(next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
    MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}


















void planner_recalculate() {   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); 
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
}




#ifdef AUTOTEMP
void getHighESpeed()
{
  static float oldt=0;
  if(!autotemp_enabled){
    return;
  }
  if(degTargetHotend0()+2<autotemp_min) {  
    return; 
  }

  float high=0.0;
  uint8_t block_index = block_buffer_tail;

  while(block_index != block_buffer_head) {
    if((block_buffer[block_index].steps_x != 0) ||
      (block_buffer[block_index].steps_y != 0) ||
      (block_buffer[block_index].steps_z != 0)) {
      float se=(float(block_buffer[block_index].steps_e)/float(block_buffer[block_index].step_event_count))*block_buffer[block_index].nominal_speed;
      
      if(se>high)
      {
        high=se;
      }
    }
    block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
  }

  float g=autotemp_min+high*autotemp_factor;
  float t=g;
  if(t<autotemp_min)
    t=autotemp_min;
  if(t>autotemp_max)
    t=autotemp_max;
  if(oldt>t)
  {
    t=AUTOTEMP_OLDWEIGHT*oldt+(1-AUTOTEMP_OLDWEIGHT)*t;
  }
  oldt=t;
  setTargetHotend0(t);
}
#endif

void check_axes_activity()
{
  unsigned char x_active = 0;
  unsigned char y_active = 0;  
  unsigned char z_active = 0;
  unsigned char e_active = 0;
  unsigned char tail_fan_speed = fanSpeed;
  #ifdef BARICUDA
  unsigned char tail_valve_pressure = ValvePressure;
  unsigned char tail_e_to_p_pressure = EtoPPressure;
  #endif
  block_t *block;

  if(block_buffer_tail != block_buffer_head)
  {
    uint8_t block_index = block_buffer_tail;
    tail_fan_speed = block_buffer[block_index].fan_speed;
    #ifdef BARICUDA
    tail_valve_pressure = block_buffer[block_index].valve_pressure;
    tail_e_to_p_pressure = block_buffer[block_index].e_to_p_pressure;
    #endif
    while(block_index != block_buffer_head)
    {
      block = &block_buffer[block_index];
      if(block->steps_x != 0) x_active++;
      if(block->steps_y != 0) y_active++;
      if(block->steps_z != 0) z_active++;
      if(block->steps_e != 0) e_active++;
      block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
    }
  }
  if((DISABLE_X) && (x_active == 0)) disable_x();
  if((DISABLE_Y) && (y_active == 0)) disable_y();
  if((DISABLE_Z) && (z_active == 0)) disable_z();
  if((DISABLE_E) && (e_active == 0))
  {
    disable_e0();
    disable_e1();
    disable_e2(); 
  }
#if defined(FAN_PIN) && FAN_PIN > -1
  #ifdef FAN_KICKSTART_TIME
    static unsigned long fan_kick_end;
    if (tail_fan_speed) {
      if (fan_kick_end == 0) {
        
        fan_kick_end = millis() + FAN_KICKSTART_TIME;
        tail_fan_speed = 255;
      } else if (fan_kick_end > millis())
        
        tail_fan_speed = 255;
    } else {
      fan_kick_end = 0;
    }
  #endif
  #ifdef FAN_SOFT_PWM
  fanSpeedSoftPwm = tail_fan_speed;
  #else
  analogWrite(FAN_PIN,tail_fan_speed);
  #endif
#endif
#ifdef AUTOTEMP
  getHighESpeed();
#endif

#ifdef BARICUDA
  #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
      analogWrite(HEATER_1_PIN,tail_valve_pressure);
  #endif

  #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
      analogWrite(HEATER_2_PIN,tail_e_to_p_pressure);
  #endif
#endif
}


float junction_deviation = 0.1;



void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder)
{
  
  int next_buffer_head = next_block_index(block_buffer_head);

  
  
  while(block_buffer_tail == next_buffer_head)
  {
    manage_heater(); 
    manage_inactivity(); 
    lcd_update();
  }

  
  
  
  long target[4];
  target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);

  #ifdef PREVENT_DANGEROUS_EXTRUDE
  if(target[E_AXIS]!=position[E_AXIS])
  {
    if(degHotend(active_extruder)<extrude_min_temp)
    {
      position[E_AXIS]=target[E_AXIS]; 
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
    }
    
    #ifdef PREVENT_LENGTHY_EXTRUDE
    if(labs(target[E_AXIS]-position[E_AXIS])>axis_steps_per_unit[E_AXIS]*EXTRUDE_MAXLENGTH)
    {
      position[E_AXIS]=target[E_AXIS]; 
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
    }
    #endif
  }
  #endif

  
  block_t *block = &block_buffer[block_buffer_head];

  
  block->busy = false;

  
#ifndef COREXY

block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
#else


block->steps_x = labs((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]));
block->steps_y = labs((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]));
#endif
  block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
  block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;
  block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

  
  if (block->step_event_count <= dropsegments)
  { 
    return; 
  }

  block->fan_speed = fanSpeed;
  #ifdef BARICUDA
  block->valve_pressure = ValvePressure;
  block->e_to_p_pressure = EtoPPressure;
  #endif

  
  block->direction_bits = 0;
#ifndef COREXY
  if (target[X_AXIS] < position[X_AXIS])
  {
    block->direction_bits |= (1<<X_AXIS); 
  }
  if (target[Y_AXIS] < position[Y_AXIS])
  {
    block->direction_bits |= (1<<Y_AXIS); 
  }
#else
  if ((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]) < 0)
  {
    block->direction_bits |= (1<<X_AXIS); 
  }
  if ((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]) < 0)
  {
    block->direction_bits |= (1<<Y_AXIS); 
  }
#endif
  if (target[Z_AXIS] < position[Z_AXIS])
  {
    block->direction_bits |= (1<<Z_AXIS); 
  }
  if (target[E_AXIS] < position[E_AXIS])
  {
    block->direction_bits |= (1<<E_AXIS); 
  }

  block->active_extruder = extruder;

  
  #ifdef COREXY
  if((block->steps_x != 0) || (block->steps_y != 0))
  {
    enable_x();
    enable_y();
  }
  #else
  if(block->steps_x != 0) enable_x();
  if(block->steps_y != 0) enable_y();
  #endif
#ifndef Z_LATE_ENABLE
  if(block->steps_z != 0) enable_z();
#endif

  
  if(block->steps_e != 0)
  {
    enable_e0();
    enable_e1();
    enable_e2(); 
  }

  if (block->steps_e == 0)
  {
    if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
  }
  else
  {
    if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
  } 

  float delta_mm[4];
  #ifndef COREXY
    delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
  #else
    delta_mm[X_AXIS] = ((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]))/axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = ((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]))/axis_steps_per_unit[Y_AXIS];
  #endif
  delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS])*extrudemultiply/100.0;
  if ( block->steps_x <=dropsegments && block->steps_y <=dropsegments && block->steps_z <=dropsegments )
  {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  } 
  else
  {
    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }
  float inverse_millimeters = 1.0/block->millimeters;  

    
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);

  
#ifdef OLD_SLOWDOWN
  if(moves_queued < (BLOCK_BUFFER_SIZE * 0.5) && moves_queued > 1)
    feed_rate = feed_rate*moves_queued / (BLOCK_BUFFER_SIZE * 0.5); 
#endif

#ifdef SLOWDOWN
  
  unsigned long segment_time = lround(1000000.0/inverse_second);
  if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5)))
  {
    if (segment_time < minsegmenttime)
    { 
      inverse_second=1000000.0/(segment_time+lround(2*(minsegmenttime-segment_time)/moves_queued));
      #ifdef XY_FREQUENCY_LIMIT
         segment_time = lround(1000000.0/inverse_second);
      #endif
    }
  }
#endif
  


  block->nominal_speed = block->millimeters * inverse_second; 
  block->nominal_rate = ceil(block->step_event_count * inverse_second); 

  
  float current_speed[4];
  float speed_factor = 1.0; 
  for(int i=0; i < 4; i++)
  {
    current_speed[i] = delta_mm[i] * inverse_second;
    if(fabs(current_speed[i]) > max_feedrate[i])
      speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }

  
#ifdef XY_FREQUENCY_LIMIT
#define MAX_FREQ_TIME (1000000.0/XY_FREQUENCY_LIMIT)
  
  unsigned char direction_change = block->direction_bits ^ old_direction_bits;
  old_direction_bits = block->direction_bits;
  segment_time = lround((float)segment_time / speed_factor);
  
  if((direction_change & (1<<X_AXIS)) == 0)
  {
    x_segment_time[0] += segment_time;
  }
  else
  {
    x_segment_time[2] = x_segment_time[1];
    x_segment_time[1] = x_segment_time[0];
    x_segment_time[0] = segment_time;
  }
  if((direction_change & (1<<Y_AXIS)) == 0)
  {
    y_segment_time[0] += segment_time;
  }
  else
  {
    y_segment_time[2] = y_segment_time[1];
    y_segment_time[1] = y_segment_time[0];
    y_segment_time[0] = segment_time;
  }
  long max_x_segment_time = max(x_segment_time[0], max(x_segment_time[1], x_segment_time[2]));
  long max_y_segment_time = max(y_segment_time[0], max(y_segment_time[1], y_segment_time[2]));
  long min_xy_segment_time =min(max_x_segment_time, max_y_segment_time);
  if(min_xy_segment_time < MAX_FREQ_TIME)
    speed_factor = min(speed_factor, speed_factor * (float)min_xy_segment_time / (float)MAX_FREQ_TIME);
#endif

  
  if( speed_factor < 1.0)
  {
    for(unsigned char i=0; i < 4; i++)
    {
      current_speed[i] *= speed_factor;
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  
  float steps_per_mm = block->step_event_count/block->millimeters;
  if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); 
  }
  else
  {
    block->acceleration_st = ceil(acceleration * steps_per_mm); 
    
    if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));

#if 0  
  
  double unit_vec[3];

  unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
  unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
  unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;










  double vmax_junction = MINIMUM_PLANNER_SPEED; 


  if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {


    double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
      - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
      - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;


    if (cos_theta < 0.95) {
      vmax_junction = min(previous_nominal_speed,block->nominal_speed);

      if (cos_theta > -0.95) {

        double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); 
        vmax_junction = min(vmax_junction,
        sqrt(block->acceleration * junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) );
      }
    }
  }
#endif

  float vmax_junction = max_xy_jerk/2; 
  float vmax_junction_factor = 1.0; 
  if(fabs(current_speed[Z_AXIS]) > max_z_jerk/2) 
    vmax_junction = min(vmax_junction, max_z_jerk/2);
  if(fabs(current_speed[E_AXIS]) > max_e_jerk/2) 
    vmax_junction = min(vmax_junction, max_e_jerk/2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
    
    vmax_junction = block->nominal_speed;
    
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/jerk);
    } 
    if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor= min(vmax_junction_factor, (max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    } 
    if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    } 
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); 
  }
  block->max_entry_speed = vmax_junction;

  
  double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);









  if (block->nominal_speed <= v_allowable) { 
    block->nominal_length_flag = true; 
  }
  else { 
    block->nominal_length_flag = false; 
  }
  block->recalculate_flag = true; 

  
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); 
  previous_nominal_speed = block->nominal_speed;


#ifdef ADVANCE
  
  if((block->steps_e == 0) || (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)) {
    block->advance_rate = 0;
    block->advance = 0;
  }
  else {
    long acc_dist = estimate_acceleration_distance(0, block->nominal_rate, block->acceleration_st);
    float advance = (STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K) * 
      (current_speed[E_AXIS] * current_speed[E_AXIS] * EXTRUTION_AREA * EXTRUTION_AREA)*256;
    block->advance = advance;
    if(acc_dist == 0) {
      block->advance_rate = 0;
    } 
    else {
      block->advance_rate = advance / (float)acc_dist;
    }
  }
  
#endif 

  calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed,
  safe_speed/block->nominal_speed);

  
  block_buffer_head = next_buffer_head;

  
  memcpy(position, target, sizeof(target)); 

  planner_recalculate();

  st_wake_up();
}

void plan_set_position(const float &x, const float &y, const float &z, const float &e)
{
  position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  position[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);
  previous_nominal_speed = 0.0; 
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
}

void plan_set_e_position(const float &e)
{
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
  st_set_e_position(position[E_AXIS]);
}

uint8_t movesplanned()
{
  return (block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
}

#ifdef PREVENT_DANGEROUS_EXTRUDE
void set_extrude_min_temp(float temp)
{
  extrude_min_temp=temp;
}
#endif


void reset_acceleration_rates()
{
	for(int8_t i=0; i < NUM_AXIS; i++)
        {
        axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        }
}
