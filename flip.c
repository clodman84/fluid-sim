#include "flip.h"
#include <math.h>
#include <raymath.h>
#include <stdio.h>
#include <stdlib.h>

#define N_PARTICLES_PER_CELL 8

float random_float() {
  float r = (float)random() / RAND_MAX;
  return r;
}

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
          Particle p = {{random_float() + i, random_float() + j},
                        {random_float(), random_float()}};
          particles[p_index] = p;
          p_index++;
        }
      }
    }
  }

  float *velocities =
      calloc(width * height * 2 + width + height, sizeof(float));
  Cell *cells = malloc(sizeof(Cell) * width * height);

  // initialise cells
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int idx = i * width + j;
      Cell *cell = &cells[idx];
      cell->type = initial_config[idx];
      // u velocities (horizontal component)
      int u_index_left = i * (width + 1) + j;
      int u_index_right = i * (width + 1) + (j + 1);
      // v velocities (vertical component)
      int v_index_down = j * (height + 1) + i;
      int v_index_up = j * (height + 1) + (i + 1);

      cell->left = &velocities[u_index_left];
      cell->right = &velocities[u_index_right];
      cell->down = &velocities[(width + 1) * height + v_index_down];
      cell->up = &velocities[(width + 1) * height + v_index_up];
    }
  }
  printf("Number of Particles: %d\n", p_index);
  ParticleSet p_set = {p_index, p_index, particles};
  Grid grid = {velocities, cells, width, height};
  Simulation sim = {p_set, grid};
  return sim;
}

void compute(Simulation *sim, Vector2 a, float dt) {
  float restitution = -1;
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

  // check how many cells are fluid
  int **n_particle_bin = (int **)calloc(sim->grid.height, sizeof(int *));
  for (int i = 0; i < sim->grid.height; i++) {
    n_particle_bin[i] = calloc(sim->grid.width, sizeof(int));
  }
  for (int i = 0; i < sim->particles.size; i++) {
    // this is what mitxela does
    int grid_x = floorf(sim->particles.data[i].position.x);
    int grid_y = floorf(sim->particles.data[i].position.y);
    if (grid_x >= 0 && grid_x < sim->grid.width && grid_y >= 0 &&
        grid_y < sim->grid.height) {
      n_particle_bin[grid_y][grid_x]++;
    }
  }
  for (int i = 0; i < sim->grid.height; i++) {
    for (int j = 0; j < sim->grid.width; j++) {
      if (n_particle_bin[i][j] >= 1)
        sim->grid.cells[i * sim->grid.width + j].type = FLUID;
      else
        sim->grid.cells[i * sim->grid.width + j].type = AIR;
    }
  }
  free(n_particle_bin);
}
