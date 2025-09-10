#include "flip.h"
#include <math.h>
#include <raymath.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N_PARTICLES_PER_CELL 8

float random_float() {
  float r = (float)random() / RAND_MAX;
  return r;
}

int get_u_index(int i, int j, int width, int height) {
  return i * (width + 1) + j;
}

int get_v_index(int i, int j, int width, int height) { return i * width + j; }

Simulation initiallise_simulation(int width, int height,
                                  enum cell_type *initial_config) {
  // do a linear search and find out exactly how many particles we need
  int n_cells = 0;
  for (int i = 0; i < width * height; i++) {
    if (initial_config[i] == FLUID)
      n_cells++;
  }
  Particle *particles =
      malloc(sizeof(Particle) * n_cells * N_PARTICLES_PER_CELL);

  int p_index = 0;
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      if (initial_config[i * width + j] == FLUID) {
        for (int n = 0; n < N_PARTICLES_PER_CELL; n++) {
          Particle p = {{random_float() + i, random_float() + j}, {0, 0}};
          particles[p_index] = p;
          p_index++;
        }
      }
    }
  }
  // u-velocities: horizontal faces
  // v-velocities: vertical faces
  float *u_velocities = calloc((width + 1) * height, sizeof(float));
  float *v_velocities = calloc(width * (height + 1), sizeof(float));

  Cell *cells = malloc(sizeof(Cell) * width * height);

  // Initialize cells with correct velocity pointers
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int cell_idx = i * width + j;
      Cell *cell = &cells[cell_idx];

      cell->type = initial_config[cell_idx];

      // Set up velocity pointers for each face
      cell->u_left = &u_velocities[get_u_index(i, j, width, height)];
      cell->u_right = &u_velocities[get_u_index(i, j + 1, width, height)];
      cell->v_bottom = &v_velocities[get_v_index(i, j, width, height)];
      cell->v_top = &v_velocities[get_v_index(i + 1, j, width, height)];
    }
  }

  printf("Number of Particles: %d\n", p_index);
  ParticleSet p_set = {p_index, p_index, particles};
  Grid grid = {u_velocities, v_velocities, cells, width, height};
  Simulation sim = {p_set, grid};
  return sim;
}

void compute(Simulation *sim, Vector2 a, float dt) {
  float restitution = -1.;
  Vector2 max = {SIMWIDTH, SIMHEIGHT};
  Vector2 min = {0, 0};

  // add gravity
  for (int i = 0; i < sim->particles.size; i++) {
    Particle *p = &sim->particles.data[i];
    p->velocity = Vector2Add(p->velocity, Vector2Scale(a, dt));
    p->position = Vector2Add(p->position, Vector2Scale(p->velocity, dt));
    if (p->position.x >= SIMWIDTH || p->position.x < 0) {
      p->velocity.x = restitution * p->velocity.x;
    };
    if (p->position.y >= SIMHEIGHT || p->position.y < 0) {
      p->velocity.y = restitution * p->velocity.y;
    };
    p->position = Vector2Clamp(p->position, min, max);
  }

  // Particle to Grid
  int *n_particle_bin = calloc(sim->grid.width * sim->grid.height, sizeof(int));
  memset(sim->grid.u_velocities, 0, sim->grid.width + 1);
  memset(sim->grid.v_velocities, 0, sim->grid.height + 1);

  for (int i = 0; i < sim->particles.size; i++) {
    // this is what mitxela does
    int grid_x = floorf(sim->particles.data[i].position.x);
    int grid_y = floorf(sim->particles.data[i].position.y);
    if (grid_x >= 0 && grid_x < sim->grid.width && grid_y >= 0 &&
        grid_y < sim->grid.height) {
      n_particle_bin[grid_y * sim->grid.width + grid_x]++;
    }
  }
  for (int i = 0; i < sim->grid.height; i++) {
    for (int j = 0; j < sim->grid.width; j++) {
      if (n_particle_bin[i * sim->grid.width + j] >= 1)
        sim->grid.cells[i * sim->grid.width + j].type = FLUID;
      else
        sim->grid.cells[i * sim->grid.width + j].type = AIR;
    }
  }
  free(n_particle_bin);
}
