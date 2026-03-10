#include "flip.h"
#include <math.h>
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>
#include <string.h>

#define N_PARTICLES_PER_CELL 16
#define FLIP_BLEND 0.99f
#define PRESSURE_ITERS 20
#define BOUNDARY_DAMPING -0.5f

static float random_float() { return (float)random() / (float)RAND_MAX; }

static float clampf(float v, float min_v, float max_v) {
  if (v < min_v)
    return min_v;
  if (v > max_v)
    return max_v;
  return v;
}

Simulation initiallise_simulation(int width, int height,
                                  enum cell_type *initial_config) {
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
          Particle p = {{j + random_float(), i + random_float()}, {0.0f, 0.0f}};
          particles[p_index++] = p;
        }
      }
    }
  }

  float *u_velocities = calloc((width + 1) * height, sizeof(float));
  float *v_velocities = calloc(width * (height + 1), sizeof(float));
  Cell *cells = malloc(sizeof(Cell) * width * height);

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int cell_idx = cell_index(i, j, width);
      Cell *cell = &cells[cell_idx];
      cell->type = initial_config[cell_idx];
      cell->u_left = &u_velocities[get_u_index(i, j, width, height)];
      cell->u_right = &u_velocities[get_u_index(i, j + 1, width, height)];
      cell->v_bottom = &v_velocities[get_v_index(i, j, width, height)];
      cell->v_top = &v_velocities[get_v_index(i + 1, j, width, height)];
      cell->divergence = 0.0f;
    }
  }

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

static void particle_to_grid(Simulation *sim) {
  int width = sim->grid.width;
  int height = sim->grid.height;
  int u_count = (width + 1) * height;
  int v_count = width * (height + 1);

  float *u_weight = calloc(u_count, sizeof(float));
  float *v_weight = calloc(v_count, sizeof(float));
  int *particle_count = calloc(width * height, sizeof(int));

  memset(sim->grid.u_velocities, 0, u_count * sizeof(float));
  memset(sim->grid.v_velocities, 0, v_count * sizeof(float));

  for (int p_idx = 0; p_idx < sim->particles.size; p_idx++) {
    Particle *p = &sim->particles.data[p_idx];
    float x = clampf(p->position.x, 0.001f, width - 0.001f);
    float y = clampf(p->position.y, 0.001f, height - 0.001f);

    int ci = (int)floorf(y);
    int cj = (int)floorf(x);
    particle_count[cell_index(ci, cj, width)]++;

    float u_x = x;
    float u_y = y - 0.5f;
    int u_i0 = (int)floorf(u_y);
    int u_j0 = (int)floorf(u_x);

    for (int di = 0; di <= 1; di++) {
      for (int dj = 0; dj <= 1; dj++) {
        int ui = u_i0 + di;
        int uj = u_j0 + dj;
        if (ui < 0 || ui >= height || uj < 0 || uj > width)
          continue;

        float wy = 1.0f - fabsf(u_y - (float)ui);
        float wx = 1.0f - fabsf(u_x - (float)uj);
        float w = fmaxf(0.0f, wx * wy);
        int index = get_u_index(ui, uj, width, height);
        sim->grid.u_velocities[index] += w * p->velocity.x;
        u_weight[index] += w;
      }
    }

    float v_x = x - 0.5f;
    float v_y = y;
    int v_i0 = (int)floorf(v_y);
    int v_j0 = (int)floorf(v_x);

    for (int di = 0; di <= 1; di++) {
      for (int dj = 0; dj <= 1; dj++) {
        int vi = v_i0 + di;
        int vj = v_j0 + dj;
        if (vi < 0 || vi > height || vj < 0 || vj >= width)
          continue;

        float wy = 1.0f - fabsf(v_y - (float)vi);
        float wx = 1.0f - fabsf(v_x - (float)vj);
        float w = fmaxf(0.0f, wx * wy);
        int index = get_v_index(vi, vj, width, height);
        sim->grid.v_velocities[index] += w * p->velocity.y;
        v_weight[index] += w;
      }
    }
  }

  for (int i = 0; i < u_count; i++) {
    if (u_weight[i] > 0.0f)
      sim->grid.u_velocities[i] /= u_weight[i];
  }
  for (int i = 0; i < v_count; i++) {
    if (v_weight[i] > 0.0f)
      sim->grid.v_velocities[i] /= v_weight[i];
  }

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      sim->grid.cells[cell_index(i, j, width)].type =
          particle_count[cell_index(i, j, width)] > 0 ? FLUID : AIR;
    }
  }

  free(u_weight);
  free(v_weight);
  free(particle_count);
}

static void add_gravity_to_grid(Simulation *sim, Vector2 a, float dt) {
  int width = sim->grid.width;
  int height = sim->grid.height;
  for (int i = 0; i <= height; i++) {
    for (int j = 0; j < width; j++) {
      int bottom_cell = i - 1;
      int top_cell = i;
      int active = 0;
      if (bottom_cell >= 0 &&
          sim->grid.cells[cell_index(bottom_cell, j, width)].type == FLUID)
        active = 1;
      if (top_cell < height &&
          sim->grid.cells[cell_index(top_cell, j, width)].type == FLUID)
        active = 1;
      if (active)
        sim->grid.v_velocities[get_v_index(i, j, width, height)] += a.y * dt;
    }
  }
}

static void enforce_boundaries(Simulation *sim) {
  int width = sim->grid.width;
  int height = sim->grid.height;

  for (int i = 0; i < height; i++) {
    sim->grid.u_velocities[get_u_index(i, 0, width, height)] = 0.0f;
    sim->grid.u_velocities[get_u_index(i, width, width, height)] = 0.0f;
  }

  for (int j = 0; j < width; j++) {
    sim->grid.v_velocities[get_v_index(0, j, width, height)] = 0.0f;
    sim->grid.v_velocities[get_v_index(height, j, width, height)] = 0.0f;
  }
}

static void make_incompressible(Simulation *sim) {
  int width = sim->grid.width;
  int height = sim->grid.height;

  for (int iter = 0; iter < PRESSURE_ITERS; iter++) {
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        Cell *cell = &sim->grid.cells[cell_index(i, j, width)];
        if (cell->type != FLUID)
          continue;

        float u_l = sim->grid.u_velocities[get_u_index(i, j, width, height)];
        float u_r =
            sim->grid.u_velocities[get_u_index(i, j + 1, width, height)];
        float v_b = sim->grid.v_velocities[get_v_index(i, j, width, height)];
        float v_t =
            sim->grid.v_velocities[get_v_index(i + 1, j, width, height)];

        float div = (u_r - u_l) + (v_t - v_b);
        float denom = 0.0f;
        if (j > 0)
          denom += 1.0f;
        if (j < width - 1)
          denom += 1.0f;
        if (i > 0)
          denom += 1.0f;
        if (i < height - 1)
          denom += 1.0f;

        if (denom <= 0.0f)
          continue;

        float corr = div / denom;
        sim->grid.u_velocities[get_u_index(i, j, width, height)] += corr;
        sim->grid.u_velocities[get_u_index(i, j + 1, width, height)] -= corr;
        sim->grid.v_velocities[get_v_index(i, j, width, height)] += corr;
        sim->grid.v_velocities[get_v_index(i + 1, j, width, height)] -= corr;

        cell->divergence = div;
      }
    }
    enforce_boundaries(sim);
  }
}

static float sample_u(const float *u, int width, int height, float x, float y) {
  float u_x = clampf(x, 0.0f, (float)width);
  float u_y = clampf(y - 0.5f, 0.0f, (float)height - 1.001f);
  int i0 = (int)floorf(u_y);
  int j0 = (int)floorf(u_x);
  int i1 = i0 + 1 < height ? i0 + 1 : i0;
  int j1 = j0 + 1 <= width ? j0 + 1 : j0;
  float fy = u_y - i0;
  float fx = u_x - j0;

  float v00 = u[get_u_index(i0, j0, width, height)];
  float v10 = u[get_u_index(i0, j1, width, height)];
  float v01 = u[get_u_index(i1, j0, width, height)];
  float v11 = u[get_u_index(i1, j1, width, height)];

  float a = v00 * (1.0f - fx) + v10 * fx;
  float b = v01 * (1.0f - fx) + v11 * fx;
  return a * (1.0f - fy) + b * fy;
}

static float sample_v(const float *v, int width, int height, float x, float y) {
  float v_x = clampf(x - 0.5f, 0.0f, (float)width - 1.001f);
  float v_y = clampf(y, 0.0f, (float)height);
  int i0 = (int)floorf(v_y);
  int j0 = (int)floorf(v_x);
  int i1 = i0 + 1 <= height ? i0 + 1 : i0;
  int j1 = j0 + 1 < width ? j0 + 1 : j0;
  float fy = v_y - i0;
  float fx = v_x - j0;

  float v00 = v[get_v_index(i0, j0, width, height)];
  float v10 = v[get_v_index(i0, j1, width, height)];
  float v01 = v[get_v_index(i1, j0, width, height)];
  float v11 = v[get_v_index(i1, j1, width, height)];

  float a = v00 * (1.0f - fx) + v10 * fx;
  float b = v01 * (1.0f - fx) + v11 * fx;
  return a * (1.0f - fy) + b * fy;
}

static void grid_to_particle(Simulation *sim, const float *prev_u,
                             const float *prev_v, float dt) {
  int width = sim->grid.width;
  int height = sim->grid.height;

  for (int p_idx = 0; p_idx < sim->particles.size; p_idx++) {
    Particle *p = &sim->particles.data[p_idx];
    float x = clampf(p->position.x, 0.001f, width - 0.001f);
    float y = clampf(p->position.y, 0.001f, height - 0.001f);

    float pic_u = sample_u(sim->grid.u_velocities, width, height, x, y);
    float pic_v = sample_v(sim->grid.v_velocities, width, height, x, y);

    float old_u = sample_u(prev_u, width, height, x, y);
    float old_v = sample_v(prev_v, width, height, x, y);

    Vector2 pic = {pic_u, pic_v};
    Vector2 flip = {p->velocity.x + (pic_u - old_u),
                    p->velocity.y + (pic_v - old_v)};

    p->velocity = Vector2Lerp(pic, flip, FLIP_BLEND);
    p->position = Vector2Add(p->position, Vector2Scale(p->velocity, dt));

    if (p->position.x <= 0.0f) {
      p->position.x = 0.001f;
      p->velocity.x *= BOUNDARY_DAMPING;
    } else if (p->position.x >= width) {
      p->position.x = width - 0.001f;
      p->velocity.x *= BOUNDARY_DAMPING;
    }

    if (p->position.y <= 0.0f) {
      p->position.y = 0.001f;
      p->velocity.y *= BOUNDARY_DAMPING;
    } else if (p->position.y >= height) {
      p->position.y = height - 0.001f;
      p->velocity.y *= BOUNDARY_DAMPING;
    }
  }
}

void compute(Simulation *sim, Vector2 a, float dt) {
  int u_count = (sim->grid.width + 1) * sim->grid.height;
  int v_count = sim->grid.width * (sim->grid.height + 1);

  particle_to_grid(sim);

  float *prev_u = malloc(sizeof(float) * u_count);
  float *prev_v = malloc(sizeof(float) * v_count);
  memcpy(prev_u, sim->grid.u_velocities, sizeof(float) * u_count);
  memcpy(prev_v, sim->grid.v_velocities, sizeof(float) * v_count);

  add_gravity_to_grid(sim, a, dt);
  make_incompressible(sim);
  grid_to_particle(sim, prev_u, prev_v, dt);

  free(prev_u);
  free(prev_v);
}
