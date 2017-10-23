/**
 *
 * Blame: Baskin Burak Senbaslar [basbursen@gmail.com]
 *
 * boids.h: boids runner ??
 *
*/


#ifndef CSSIM

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "position_external.h"


#endif


#include "boids.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "log.h"

#define BOIDS_EPSILON 0.0000001
#define BOIDS_COLAVD_SHIFT 0.4
#define BOIDS_COLAVD_THRESH 0.05
#define BOIDS_COLAVD_CLAMP 0.002

#define BOIDS_XMAX 2.5
#define BOIDS_XMIN -2.5
#define BOIDS_YMAX 2.5
#define BOIDS_YMIN -2.5
#define BOIDS_ZMAX 2
#define BOIDS_ZMIN 0.5

#ifndef CSSIM
extern xSemaphoreHandle flockLock;
#endif

struct vec leaderDest;

void init_boids(struct boid_data* data, int i, int t, struct vec dest, struct cf_status* cfs) {
/*
#ifdef CSSIM
  printf("init type: %d", t);
#endif
*/

  for(int i=0; i<0xff; i++) {
    cfs[i].last_time = 0;
  }

  data->role = t;
  if(data->role == BOIDS_LEADER)
    data->dest = dest;
  data->cfs = cfs;
  data->my_id = i;
}

/* tries to anticipate closest point to another cf, and tries to act accordingly */
static struct boid_suggestion collision_avoidance_anticipate(struct cf_status* flockstatus, struct cf_status* my_status) {
  struct boid_suggestion sgg;
  sgg.importance = 0;
  sgg.vel = vzero();
  float neigh_thresh = 2; // meters

  int fcount = 0;

  neigh_thresh = neigh_thresh * neigh_thresh;

  for(int i=0; i<0xff; i++) {
    if(flockstatus[i].id != my_status->id && flockstatus[i].last_time != 0) {
      float dist;
      if((dist = vdist2(my_status->position, flockstatus[i].position)) < neigh_thresh) {
        struct vec p = vsub(flockstatus[i].position, my_status->position);
        float dotp;
        if((dotp = vdot(p, my_status->linear_velocity)) > BOIDS_EPSILON) {
          fcount++;
          float magvel = vmag(my_status->linear_velocity);
          float closest_distance = vmag(vcross(p, my_status->linear_velocity)) / magvel;
          float closest_time = dotp / (magvel * magvel);
          float imp = 1 / (closest_distance * closest_time);
          sgg.importance += imp;

          struct vec l = vcross(p, vcross(my_status->position, p));
          l = vnormalize(l);

          sgg.vel = vadd(sgg.vel, vscl(imp, l));
        }
      }
    }
  }

  if(fcount != 0)
    sgg.vel = vnormalize(sgg.vel);

  return sgg;
}

/*tries to keep certain distance from closest flockmate */

static struct boid_suggestion collision_avoidance_closest_distance(struct cf_status* flockstatus, struct cf_status* my_status) {
  struct boid_suggestion sgg;
  sgg.importance = 0;
  sgg.vel = vzero();

  float neigh_thresh = 1;

  float closest_dist = 1e10;

  neigh_thresh = neigh_thresh * neigh_thresh;

  for(int i=0; i<0xff; i++) {
		if(flockstatus[i].id != my_status->id && flockstatus[i].last_time != 0) {
			float dist;
			if((dist = vdist2(my_status->position, flockstatus[i].position)) < neigh_thresh) {
				dist = sqrt(dist);
        if(dist < closest_dist) {
				  struct vec p = vsub(flockstatus[i].position, my_status->position);
          float dotp;
          if((dotp = vdot(p, my_status->linear_velocity)) > BOIDS_EPSILON) {
					  closest_dist = dist;
            sgg.vel = vnormalize(vcross(p, vcross(my_status->linear_velocity, p)));
            sgg.importance = 1 / closest_dist;
				  }
        }
			}
		}
	}

  return sgg; 
}


/* tries to keep certain distance from flockmates*/
static struct boid_suggestion collision_avoidance_distance(struct cf_status* flockstatus, struct cf_status* my_status) {
	struct boid_suggestion sgg;
	sgg.importance = 0;
	sgg.vel = vzero();
	float neigh_thresh = 1;// meters

	float total_dist = 0;
	float closest_dist = 1e10;

	float w = 0;

	int fcount = 0;

	neigh_thresh = neigh_thresh * neigh_thresh;

	for(int i=0; i<0xff; i++) {
		if(flockstatus[i].id != my_status->id && flockstatus[i].last_time != 0) {
			float dist;
			if((dist = vdist2(my_status->position, flockstatus[i].position)) < neigh_thresh) {
				dist = sqrt(dist);
				closest_dist = fmin(closest_dist, dist);
				float weight;
				if(dist < BOIDS_COLAVD_SHIFT + BOIDS_COLAVD_THRESH) {
					weight = 1/(BOIDS_COLAVD_CLAMP*BOIDS_COLAVD_CLAMP);
				} else {
					weight = 1/((dist-BOIDS_COLAVD_SHIFT)*(dist-BOIDS_COLAVD_SHIFT));
				}
				struct vec p = vsub(flockstatus[i].position, my_status->position);
				float dotp;
				if((dotp = vdot(p, my_status->linear_velocity)) > BOIDS_EPSILON) {
					total_dist += dist;
					fcount++;
					struct vec l = vnormalize(vcross(p, vcross(my_status->linear_velocity, p)));
					
					sgg.vel = vadd(sgg.vel, vscl(weight, l)); 
					w += weight;
				}
			}
		}
	}

	if(fcount > 0) {
		sgg.vel = vdiv(sgg.vel, w);
		sgg.importance = 1 / (closest_dist);
		//sgg.importance = total_dist / fcount;
	}
/*
#ifdef CSSIM
	printf("colavd || sgg.vel %f %f %f sgg.importance %f\n", sgg.vel.x, sgg.vel.y, sgg.vel.z, sgg.importance);
#endif
*/
	return sgg;
}

static struct boid_suggestion collision_avoidance_goopposite(struct cf_status* flockstatus, struct cf_status* my_status) {
	struct boid_suggestion sgg;
	sgg.importance = 0;
	sgg.vel = vzero();
	float neigh_thresh = 10; // meters

	float closest_dist = 1e10;

	float w = 0;


	int fcount = 0;
	neigh_thresh = neigh_thresh * neigh_thresh;

	for(int i=0; i<0xff; i++) {
		if(flockstatus[i].id != my_status->id && flockstatus[i].last_time !=0 ) {
			float dist;
			if((dist = vdist2(my_status->position, flockstatus[i].position)) < neigh_thresh) {
				fcount++;
				dist = sqrt(dist);
				closest_dist = fmin(closest_dist, dist);
				struct vec l = vsub(my_status->position, flockstatus[i].position);
				sgg.vel = vadd(sgg.vel, l);
				w += dist;
			}
		}
	}

	if(fcount > 0) {
		sgg.vel = vdiv(sgg.vel, w);
		sgg.importance = 1/closest_dist;
	}

	return sgg;
}

static struct boid_suggestion flock_centering(struct cf_status* flockstatus, struct cf_status* my_status) {
  struct boid_suggestion sgg;
  sgg.importance = 0;
  sgg.vel = vzero();

  float neigh_thresh = 1.5; // meters

  float w = 0;

  neigh_thresh = neigh_thresh * neigh_thresh;

  for(int i=0; i<0xff; i++) {
    if(flockstatus[i].id != my_status->id && flockstatus[i].last_time != 0) {
      float dist;
      if((dist = vdist2(my_status->position, flockstatus[i].position)) < neigh_thresh) {
        dist = sqrt(dist);
				//dist = 1/dist;
        sgg.vel = vadd(sgg.vel, vscl(dist*dist*dist*dist*dist*dist, flockstatus[i].position));
        w += dist*dist*dist*dist*dist*dist;
      }
    }
  }

  if(w > BOIDS_EPSILON) {
    sgg.vel = vdiv(sgg.vel, w);
    sgg.vel = vsub(sgg.vel, my_status->position);
    sgg.importance = vmag(sgg.vel);
    sgg.vel = vnormalize(sgg.vel);
  }

  return sgg;
}

static struct boid_suggestion velocity_matching(struct cf_status* flockstatus, struct cf_status* my_status) {
  struct boid_suggestion sgg;
  sgg.importance = 0;
  sgg.vel = vzero();

  float neigh_thresh = 1;
  neigh_thresh = neigh_thresh * neigh_thresh;

  int fcount = 0;

  for(int i=0; i<0xff; i++) {
    if(i != my_status->id && flockstatus[i].last_time != 0) {
      float dist;
      if((dist = vdist2(my_status->position, flockstatus[i].position)) < neigh_thresh) {
        sgg.vel = vadd(sgg.vel, flockstatus[i].linear_velocity);
        fcount++;
      }
    }
  }

  if(fcount != 0) {
    sgg.vel = vdiv(sgg.vel, fcount);
    float sggmag = vmag(sgg.vel), mymag = vmag(my_status->linear_velocity);
    if(mymag < BOIDS_EPSILON || sggmag < BOIDS_EPSILON)
      sgg.importance = 1;
    else {
      sgg.importance = fabs(sggmag - mymag) * vmag(vcross(sgg.vel, my_status->linear_velocity)) / (sggmag * mymag);
		}
    sgg.vel = vnormalize(sgg.vel);
  }
 /*
#ifdef CSSIM
	printf("velmat || sgg.vel %f %f %f sgg.importance %f\n", sgg.vel.x, sgg.vel.y, sgg.vel.z, sgg.importance);
#endif
*/
  return sgg;
}

static struct boid_suggestion boid_go(struct boid_data* data, struct cf_status* my_status) {
  struct boid_suggestion sgg;

  struct vec diff = vsub(data->dest, my_status->position);
  sgg.importance = fmin(2.0, vmag(diff));
/*
#ifdef CSSIM
  printf("%f %f %f\n",diff.x, diff.y, diff.z);
#endif
*/
  sgg.vel = vnormalize(diff);
/*
#ifdef CSSIM
  printf("%f %f %f\n",sgg.vel.x, sgg.vel.y, sgg.vel.z);
#endif
*/

  return sgg;
}

static struct boid_suggestion empty_sgg(struct cf_status* flockstatus, struct cf_status* my_status) {
	struct boid_suggestion sgg;
	sgg.vel = vzero();
	sgg.importance = 0;
	return sgg;
}

struct traj_eval eval_boids(struct boid_data *data, float t) {
    /*struct traj_eval ev2;

    struct cf_status* my_status2;

    my_status2 = &data->cfs[data->my_id];

	ev2.pos = my_status2->position;
	ev2.vel = vzero();



	ev2.acc = vzero();
	ev2.yaw = 0;
	ev2.omega = vzero();
    


    return ev2;*/






	struct boid_suggestion (*colavd)(struct cf_status*, struct cf_status*);
	struct boid_suggestion (*velmat)(struct cf_status*, struct cf_status*);
	struct boid_suggestion (*flocen)(struct cf_status*, struct cf_status*);
	struct boid_suggestion (*gofn)(struct boid_data*, struct cf_status*);


	colavd = &collision_avoidance_closest_distance;
	velmat = &velocity_matching;
	flocen = &flock_centering;
	gofn = &boid_go;

  /*
  struct traj_eval ev;
	ev.pos = at->pos;
	ev.vel = at->vel;
	ev.acc = vzero();
	ev.yaw = 0;
	ev.omega = vzero();
  */



#ifndef CSSIM
  xSemaphoreTake(flockLock, portMAX_DELAY);
#endif

 /* for(int i=0; i<0xff; i++) {
    if(data->cfs[i].last_time != 0) {
      float dt = t - data->cfs[i].last_time;
      (data->cfs[i]).position = vadd((data->cfs[i]).position, vscl(dt, (data->cfs[i]).linear_velocity));
      data->cfs[i].last_time = t;
    }
  }*/

  struct cf_status* my_status;
  struct traj_eval trj;

  /*
  // collision_avoidance_distance
  float ca_imp = 0.076;
  float fc_imp = 0.06;
  float vm_imp = 0.025;
  float go_imp = 0.035;
  */

  // collision_avoidance_closest_distance
  float ca_imp = 0.096;
  float fc_imp = 0.06;
  float vm_imp = 0.025;
  float go_imp = 0.035;


  struct boid_suggestion sgg;





  my_status = &data->cfs[data->my_id];
  

#ifdef CSSIM
	float min_dist = 1e10;
	for(int i=0; i<0xff; i++) {
		if(data->cfs[i].id != data->my_id && data->cfs[i].last_time != 0) {
			float dist = vdist(my_status->position, (data->cfs)[i].position);
			min_dist = fmin(dist, min_dist);
		}
	}

	printf("%f\n", min_dist);

#endif


  sgg = (*colavd)(data->cfs, my_status);
  trj.vel = vscl(ca_imp * sgg.importance, sgg.vel);


#ifdef CSSIM
	printf("ca> %f %f %f %f, ", sgg.vel.x, sgg.vel.y, sgg.vel.z, sgg.importance);
#endif


  sgg = (*flocen)(data->cfs, my_status);
  trj.vel = vadd(trj.vel, vscl(fc_imp * sgg.importance, sgg.vel));


#ifdef CSSIM
	printf("fc> %f %f %f %f, ", sgg.vel.x, sgg.vel.y, sgg.vel.z, sgg.importance);
#endif


  sgg = (*velmat)(data->cfs, my_status);
  trj.vel = vadd(trj.vel, vscl(vm_imp * sgg.importance, sgg.vel));


#ifdef CSSIM
	printf("vm> %f %f %f %f, ", sgg.vel.x, sgg.vel.y, sgg.vel.z, sgg.importance);
#endif


  if(data->role == BOIDS_LEADER) {
    leaderDest = data->dest;
    sgg = (*gofn)(data, my_status);
    trj.vel = vadd(trj.vel, vscl(go_imp * sgg.importance, sgg.vel));


#ifdef CSSIM
		printf("go> %f %f %f %f, ", sgg.vel.x, sgg.vel.y, sgg.vel.z, sgg.importance);
#endif


  }

#ifdef CSSIM
	printf("result> %f %f %f", trj.vel.x, trj.vel.y, trj.vel.z);
#endif

#ifdef CSSIM
	if(vmag(trj.vel) > 1.5) {
		printf(" !!!!!!!!!!!!!!!!!!!! \n");
	} else printf("\n");
#endif

#ifndef CSSIM
  xSemaphoreGive(flockLock);
#endif

 


  if(my_status->position.x - BOIDS_XMAX > 0 && trj.vel.x > 0) {
    trj.vel.x = 0;
  }

  if(my_status->position.x - BOIDS_XMIN < 0 && trj.vel.x < 0) {
    trj.vel.x = 0;
  }

  if(my_status->position.z - BOIDS_ZMAX > 0 && trj.vel.z > 0) {
    trj.vel.z = 0;
  }

  if(my_status->position.z - BOIDS_ZMIN < 0 && trj.vel.z < 0) {
    trj.vel.z = 0;
  }

  if(my_status->position.y - BOIDS_YMAX > 0 && trj.vel.y > 0) {
    trj.vel.y = 0;
  }

  if(my_status->position.y - BOIDS_YMIN < 0 && trj.vel.y < 0) {
    trj.vel.y = 0;
  }


 /* if(vmag(trj.vel) > 1)
    trj.vel = vnormalize(trj.vel);*/

/*
#ifdef CSSIM
	printf("%f\n", vmag(trj.vel));
#endif
*/
  //trj.vel = vnormalise(trj.vel);
  trj.pos = vadd(my_status->position, vscl(1/500.0, trj.vel));
  trj.acc = vzero();
  trj.yaw = 0;
  trj.omega = vzero();
/*
#ifdef CSSIM
  printf("id: %d, pos: %f %f %f, vel: %f %f %f\n", data->my_id, trj.pos.x, trj.pos.y, trj.pos.z, trj.vel.x, trj.vel.y, trj.vel.z);
#endif
*/
  return trj;
}

#ifndef CSSIM
LOG_GROUP_START(boids)
LOG_ADD(LOG_FLOAT, leaderX, &leaderDest.x)
LOG_ADD(LOG_FLOAT, leaderY, &leaderDest.y)
LOG_ADD(LOG_FLOAT, leaderZ, &leaderDest.z)
LOG_GROUP_STOP(boids)
#endif
