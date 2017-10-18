#pragma once

#include "math3d.h"
#include <stdint.h>

struct cf_status {
  uint8_t id;
  struct vec position; 
  struct quat orientation;
  struct vec linear_velocity;
  float last_time;
};



//for sim only
struct cf_status* get_cfstat_array(); 
void init_cfstat_array();
void update_cfstat_position(int id, float x, float y, float z, float t);
void update_cfstat_velocity(int id, float x, float y, float z, float t);

