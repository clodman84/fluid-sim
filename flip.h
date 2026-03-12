#ifndef FLIP_H
#define FLIP_H
#include <raylib.h>

#define CELL_SIZE 10.0
#define SIMWIDTH 80
#define SIMHEIGHT 80

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
