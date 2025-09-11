#ifndef FLIP_H
#define FLIP_H
#include <raylib.h>

#define CELL_SIZE 100.0
#define SIMWIDTH 8
#define SIMHEIGHT 8

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

int get_u_index(int i, int j, int width, int height);
int get_v_index(int i, int j, int width, int height);

#endif
