#include "cf_status.h"
#include <stdlib.h>

struct cf_status* cfstt;

void init_cfstat_array() {
  cfstt = (struct cf_status*) malloc(sizeof(struct cf_status) * 0xff);
  for(int i=0; i<0xff; i++) {
    cfstt[i].last_time = 0;
    cfstt[i].id = i;
  }
}

struct cf_status* get_cfstat_array() {
  return cfstt;
}

void update_cfstat_position(int id, float x, float y, float z, float t) {
  cfstt[id].position.x = x;
  cfstt[id].position.y = y;
  cfstt[id].position.z = z;
  cfstt[id].last_time = t;
}
void update_cfstat_velocity(int id, float x, float y, float z, float t) {
  cfstt[id].linear_velocity.x = x;
  cfstt[id].linear_velocity.y = y;
  cfstt[id].linear_velocity.z = z;
  cfstt[id].last_time = t;
}
