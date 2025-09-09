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
  float *up;
  float *down;
  float *left;
  float *right;
  enum cell_type type;
} Cell;

typedef struct {
  float *grid_velocities;
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

#endif
