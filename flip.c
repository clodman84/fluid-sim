#include "flip.h"
#include <math.h>
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
  // u-velocities: horizontal faces
  // v-velocities: vertical faces
  float *u_velocities = calloc((width + 1) * height, sizeof(float));
  float *v_velocities = calloc(width * (height + 1), sizeof(float));

  Cell *cells = malloc(sizeof(Cell) * width * height);

  // Initialize cells with correct velocity pointers
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int cell_idx = cell_index(i, j, width);
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

void destroy_sim(Simulation *sim) {
  free(sim->grid.cells);
  free(sim->grid.u_velocities);
  free(sim->grid.v_velocities);
  free(sim->particles.data);
}

void add_gravity(Simulation *sim, Vector2 a, float dt) {
  float restitution = -1.0;
  Vector2 max = {SIMWIDTH, SIMHEIGHT};
  Vector2 min = {0, 0};
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
}

void particle_to_grid(Simulation *sim) {
  int *n_particle_bin = calloc(sim->grid.width * sim->grid.height, sizeof(int));
  memset(sim->grid.u_velocities, 0,
         (sim->grid.width + 1) * sim->grid.height * sizeof(float));
  memset(sim->grid.v_velocities, 0,
         sim->grid.width * (sim->grid.height + 1) * sizeof(float));

  for (int i = 0; i < sim->particles.size; i++) {
    // this is what mitxela does
    int grid_x = floorf(sim->particles.data[i].position.x);
    int grid_y = floorf(sim->particles.data[i].position.y);

    // <WHY DO I HAVE TO DO THIS!! IS THERE A WAY TO REMOVE THIS>
    if (grid_x < 0 || grid_x >= sim->grid.width || grid_y < 0 ||
        grid_y >= sim->grid.height) {
      continue;
    }

    int cell_idx = cell_index(grid_x, grid_y, sim->grid.width);
    Cell *cell = &sim->grid.cells[cell_idx];

    float dx = sim->particles.data[i].position.x - grid_x;
    float dy = sim->particles.data[i].position.y - grid_y;

    if (grid_x == 0)
      dx = 0;
    if (grid_x == SIMWIDTH - 1)
      dx = 1;

    float u_left = (1 - dx) * sim->particles.data[i].velocity.x;
    float u_right = dx * sim->particles.data[i].velocity.x;
    float v_top = (1 - dy) * sim->particles.data[i].velocity.y;
    float v_bottom = dy * sim->particles.data[i].velocity.y;

    *cell->u_right += u_right;
    *cell->u_left += u_left;
    *cell->v_bottom += v_bottom;
    *cell->v_top += v_top;

    n_particle_bin[cell_idx]++;
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

void make_incompressible(Simulation *sim) {
  for (int i = 0; i < sim->grid.height; i++) {
    for (int j = 0; j < sim->grid.width; j++) {
      Cell *cell = &sim->grid.cells[i * sim->grid.width + j];
      float d = *cell->u_left + *cell->v_bottom - *cell->u_right - *cell->v_top;
      float denominator = 4.0;

      if (i == 0 || i == SIMHEIGHT - 1)
        denominator -= 1.0;
      if (j == 0 || j == SIMWIDTH - 1)
        denominator -= 1.0;

      float adjustment = d / denominator;
      printf("%d, %d; u_left: %f, u_right: %f, Divergence: %f, Denominator: "
             "%f, Adjustment: "
             "%f --> ",
             i, j, *cell->u_left, *cell->u_right, d, denominator, adjustment);

      *cell->u_left -= adjustment;
      *cell->v_bottom -= adjustment;
      *cell->u_right += adjustment;
      *cell->v_top += adjustment;

      // REMOVE THIS IF IT WORKS, ONLY FOR TESTING PURPOSES
      d = *cell->u_left + *cell->v_bottom - *cell->u_right - *cell->v_top;
      printf("Divergence: %f\n", d);
      cell->divergence = d;
    }
  }
}

void compute(Simulation *sim, Vector2 a, float dt) {
  add_gravity(sim, a, dt);
  particle_to_grid(sim);
  // make_incompressible(sim);
}
