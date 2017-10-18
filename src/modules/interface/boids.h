#pragma once
/**
 *
 * Blame: Baskin Burak Senbaslar [basbursen@gmail.com]
 *
 * boids.h: boids runner ??
 *
*/
#include "math3d.h"
#include "pptraj.h"
#include <stdint.h>
#include "cf_status.h"

enum boids_role {
  BOIDS_FOLLOWER = 0,
  BOIDS_LEADER = 1,
};

struct boid_suggestion {
  struct vec vel;
  float importance;
};

struct boid_data {
  int my_id;
  struct vec dest;
  int role; // 0 follower: follow the flock, 1 leader: try to go to dest
  struct cf_status* cfs;
};

void init_boids(struct boid_data* data, int i, int t, struct vec dest, struct cf_status* cfs);
struct traj_eval eval_boids(struct boid_data* dst, float t);
