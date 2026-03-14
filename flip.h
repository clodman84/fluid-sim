#ifndef FLIP_H
#define FLIP_H
#include <raylib.h>

#define CELL_SIZE 100
#define SIMWIDTH 8
#define SIMHEIGHT 8
#define N_PARTICLES_PER_CELL 4
#define FLIP_BLEND .95f
#define PRESSURE_ITERS 4
#define BOUNDARY_DAMPING -1.0f
#define PARTICLE_COLLISION_ITERS 2
#define PARTICLE_RADIUS .25f
#define DRIFT_COMPENSATION 1.0f
#define OVERRELAXATION 1.9f

enum cell_type { AIR, FLUID };

typedef struct {
  Vector2 position;
  Vector2 velocity;
} Particle;

typedef struct {
  int size;
  int max_size;
  Particle *data;
} ParticleSet;

typedef struct {
  float *v_top;
  float *v_bottom;
  float *u_left;
  float *u_right;
  float divergence; // remove this later ONLY FOR TESTING PURPOSES
  float pressure;
  float density;
  enum cell_type type;
} Cell;

typedef struct {
  float *u_velocities;
  float *v_velocities;
  Cell *cells;
  int width;
  int height;
} Grid;

typedef struct {
  ParticleSet particles;
  Grid grid;

  // Scratch buffers reused between simulation steps to avoid per-frame
  // allocations in hot paths.
  float *u_weight;
  float *v_weight;
  int *particle_count;
  float *pressure;
  float *pressure_next;
  float *divergence;
  float *prev_u;
  float *prev_v;

  int *collision_next;
  int *collision_cell_x;
  int *collision_cell_y;
  int collision_bucket_count;
  int collision_particle_capacity;
} Simulation;

Simulation initiallise_simulation(int width, int height, enum cell_type *cells);
void compute(Simulation *sim, Vector2 a, float dt);
void destroy_sim(Simulation *sim);

static inline int get_u_index(int i, int j, int width, int height) {
  return i * (width + 1) + j;
}

static inline int get_v_index(int i, int j, int width, int height) {
  return i * width + j;
}

static inline int cell_index(int i, int j, int width) { return i * width + j; }

#endif
