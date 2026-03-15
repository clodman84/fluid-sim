#include "flip.h"
#include <limits.h>
#include <math.h>
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>
#include <string.h>

static float random_float() { return (float)random() / (float)RAND_MAX; }

static float clampf(float v, float min_v, float max_v) {
  if (v < min_v)
    return min_v;
  if (v > max_v)
    return max_v;
  return v;
}

static void resolve_particle_pair(Particle *a, Particle *b, float min_dist,
                                  int width, int height) {
  Vector2 delta = Vector2Subtract(b->position, a->position);
  float d2 = Vector2LengthSqr(delta);
  if (d2 <= 1e-10f) {
    float angle = random_float() * 2.0f * PI;
    delta = (Vector2){cosf(angle), sinf(angle)};
    d2 = 1.0f;
  }

  float min_d2 = min_dist * min_dist;
  if (d2 >= min_d2)
    return;

  float d = sqrtf(d2);
  Vector2 n = Vector2Scale(delta, 1.0f / d);
  float corr = 0.5f * (min_dist - d);
  a->position = Vector2Subtract(a->position, Vector2Scale(n, corr));
  b->position = Vector2Add(b->position, Vector2Scale(n, corr));

  a->position.x = clampf(a->position.x, 0.001f, width - 0.001f);
  a->position.y = clampf(a->position.y, 0.001f, height - 0.001f);
  b->position.x = clampf(b->position.x, 0.001f, width - 0.001f);
  b->position.y = clampf(b->position.y, 0.001f, height - 0.001f);
}

typedef struct {
  int key;
  int head;
} HashBucket;

static int hash_coords(int x, int y) {
  // Large primes for 2D integer hashing.
  unsigned int hx = (unsigned int)(x * 73856093);
  unsigned int hy = (unsigned int)(y * 19349663);
  return (int)(hx ^ hy);
}

static int find_or_create_bucket(HashBucket *buckets, int bucket_count,
                                 int key) {
  int slot = (key & 0x7fffffff) % bucket_count;
  while (1) {
    if (buckets[slot].key == key)
      return slot;
    if (buckets[slot].key == INT_MIN) {
      buckets[slot].key = key;
      buckets[slot].head = -1;
      return slot;
    }
    slot = (slot + 1) % bucket_count;
  }
}

static int find_bucket(const HashBucket *buckets, int bucket_count, int key) {
  int slot = (key & 0x7fffffff) % bucket_count;
  while (1) {
    if (buckets[slot].key == key)
      return slot;
    if (buckets[slot].key == INT_MIN)
      return -1;
    slot = (slot + 1) % bucket_count;
  }
}

static void resolve_particle_collisions(Simulation *sim) {
  int n = sim->particles.size;
  if (n <= 1)
    return;

  float min_dist = 2.0f * PARTICLE_RADIUS;
  float inv_cell_size = 1.0f / min_dist;

  int bucket_count = 1;
  while (bucket_count < (n * 2))
    bucket_count <<= 1;

  if (sim->collision_bucket_count != bucket_count || !sim->collision_next ||
      !sim->collision_cell_x || !sim->collision_cell_y ||
      sim->collision_particle_capacity < n) {
    free(sim->collision_next);
    free(sim->collision_cell_x);
    free(sim->collision_cell_y);

    sim->collision_next = malloc(sizeof(int) * n);
    sim->collision_cell_x = malloc(sizeof(int) * n);
    sim->collision_cell_y = malloc(sizeof(int) * n);
    sim->collision_particle_capacity = n;
    sim->collision_bucket_count = bucket_count;
  }

  if (!sim->collision_next || !sim->collision_cell_x || !sim->collision_cell_y)
    return;

  HashBucket *buckets = malloc(sizeof(HashBucket) * bucket_count);
  if (!buckets)
    return;

  for (int iter = 0; iter < PARTICLE_COLLISION_ITERS; iter++) {
    for (int i = 0; i < bucket_count; i++) {
      buckets[i].key = INT_MIN;
      buckets[i].head = -1;
    }

    for (int p_idx = 0; p_idx < n; p_idx++) {
      Particle *p = &sim->particles.data[p_idx];
      int cx = (int)floorf(p->position.x * inv_cell_size);
      int cy = (int)floorf(p->position.y * inv_cell_size);
      sim->collision_cell_x[p_idx] = cx;
      sim->collision_cell_y[p_idx] = cy;

      int key = hash_coords(cx, cy);
      int slot = find_or_create_bucket(buckets, bucket_count, key);
      sim->collision_next[p_idx] = buckets[slot].head;
      buckets[slot].head = p_idx;
    }

    for (int a_idx = 0; a_idx < n; a_idx++) {
      int ax = sim->collision_cell_x[a_idx];
      int ay = sim->collision_cell_y[a_idx];

      for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {
          int nx = ax + ox;
          int ny = ay + oy;
          int key = hash_coords(nx, ny);
          int slot = find_bucket(buckets, bucket_count, key);
          if (slot < 0)
            continue;

          for (int b_idx = buckets[slot].head; b_idx != -1;
               b_idx = sim->collision_next[b_idx]) {
            if (b_idx <= a_idx)
              continue;
            if (sim->collision_cell_x[b_idx] != nx ||
                sim->collision_cell_y[b_idx] != ny)
              continue;

            resolve_particle_pair(&sim->particles.data[a_idx],
                                  &sim->particles.data[b_idx], min_dist,
                                  sim->grid.width, sim->grid.height);
          }
        }
      }
    }
  }

  free(buckets);
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
      cell->pressure = 0.0f;
      cell->density = 0.0f;
    }
  }

  ParticleSet p_set = {p_index, p_index, particles};
  Grid grid = {u_velocities, v_velocities, cells, width, height};
  Simulation sim = {0};
  sim.particles = p_set;
  sim.grid = grid;
  return sim;
}

void destroy_sim(Simulation *sim) {
  free(sim->grid.cells);
  free(sim->grid.u_velocities);
  free(sim->grid.v_velocities);
  free(sim->particles.data);

  free(sim->u_weight);
  free(sim->v_weight);
  free(sim->particle_count);
  free(sim->pressure);
  free(sim->pressure_next);
  free(sim->divergence);
  free(sim->prev_u);
  free(sim->prev_v);

  free(sim->collision_next);
  free(sim->collision_cell_x);
  free(sim->collision_cell_y);
}

static void particle_to_grid(Simulation *sim) {
  int width = sim->grid.width;
  int height = sim->grid.height;
  int u_count = (width + 1) * height;
  int v_count = width * (height + 1);

  if (!sim->u_weight)
    sim->u_weight = calloc(u_count, sizeof(float));
  if (!sim->v_weight)
    sim->v_weight = calloc(v_count, sizeof(float));
  if (!sim->particle_count)
    sim->particle_count = calloc(width * height, sizeof(int));

  if (!sim->u_weight || !sim->v_weight || !sim->particle_count)
    return;

  memset(sim->u_weight, 0, u_count * sizeof(float));
  memset(sim->v_weight, 0, v_count * sizeof(float));
  memset(sim->particle_count, 0, width * height * sizeof(int));
  memset(sim->grid.u_velocities, 0, u_count * sizeof(float));
  memset(sim->grid.v_velocities, 0, v_count * sizeof(float));

  for (int p_idx = 0; p_idx < sim->particles.size; p_idx++) {
    Particle *p = &sim->particles.data[p_idx];
    float x = clampf(p->position.x, 0.001f, width - 0.001f);
    float y = clampf(p->position.y, 0.001f, height - 0.001f);

    int ci = (int)floorf(y);
    int cj = (int)floorf(x);
    sim->particle_count[cell_index(ci, cj, width)]++;

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
        sim->u_weight[index] += w;
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
        sim->v_weight[index] += w;
      }
    }
  }

  for (int i = 0; i < u_count; i++) {
    if (sim->u_weight[i] > 0.0f)
      sim->grid.u_velocities[i] /= sim->u_weight[i];
  }
  for (int i = 0; i < v_count; i++) {
    if (sim->v_weight[i] > 0.0f)
      sim->grid.v_velocities[i] /= sim->v_weight[i];
  }

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int idx = cell_index(i, j, width);
      sim->grid.cells[idx].type = sim->particle_count[idx] > 0 ? FLUID : AIR;
      // sim->grid.cells[idx].pressure = 0.0f;
      sim->grid.cells[idx].density =
          (float)sim->particle_count[idx] / (float)N_PARTICLES_PER_CELL;
      sim->grid.cells[idx].divergence = 0.0f;
    }
  }
}

static void add_gravity_to_grid(Simulation *sim, Vector2 a, float dt) {
  int width = sim->grid.width;
  int height = sim->grid.height;

  // v faces live on a (height + 1) x width grid.
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

  // u faces live on a height x (width + 1) grid.
  for (int i = 0; i < height; i++) {
    for (int j = 0; j <= width; j++) {
      int left_cell = j - 1;
      int right_cell = j;
      int active = 0;
      if (left_cell >= 0 &&
          sim->grid.cells[cell_index(i, left_cell, width)].type == FLUID)
        active = 1;
      if (right_cell < width &&
          sim->grid.cells[cell_index(i, right_cell, width)].type == FLUID)
        active = 1;
      if (active)
        sim->grid.u_velocities[get_u_index(i, j, width, height)] += a.x * dt;
    }
  }
}

static int is_fluid_cell(Grid *grid, int i, int j) {
  if (i < 0 || i >= grid->height || j < 0 || j >= grid->width)
    return 0;
  return grid->cells[cell_index(i, j, grid->width)].type == FLUID;
}

static void make_incompressible(Simulation *sim) {
  int width = sim->grid.width;
  int height = sim->grid.height;
  int cell_count = width * height;

  // Pressure is used as a visualization/debug scalar here. The
  // incompressibility solve itself is done directly in velocity-space.
  if (!sim->pressure)
    sim->pressure = calloc(cell_count, sizeof(float));
  if (sim->pressure)
    memset(sim->pressure, 0, cell_count * sizeof(float));

  // Velocity-space incompressibility solve: iteratively push face velocities so
  // each fluid cell's divergence approaches zero.
  for (int iter = 0; iter < GAUSS_ITERS; iter++) {
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        int idx = cell_index(i, j, width);
        Cell *cell = &sim->grid.cells[idx];
        if (cell->type != FLUID)
          continue;

        float u_l = sim->grid.u_velocities[get_u_index(i, j, width, height)];
        float u_r =
            sim->grid.u_velocities[get_u_index(i, j + 1, width, height)];
        float v_b = sim->grid.v_velocities[get_v_index(i, j, width, height)];
        float v_t =
            sim->grid.v_velocities[get_v_index(i + 1, j, width, height)];

        float divergence = (u_r - u_l) + (v_t - v_b);
        float compression = fmaxf(0.0f, cell->density - 1.0f);
        int interior = is_fluid_cell(&sim->grid, i, j - 1) &&
                       is_fluid_cell(&sim->grid, i, j + 1) &&
                       is_fluid_cell(&sim->grid, i - 1, j) &&
                       is_fluid_cell(&sim->grid, i + 1, j);
        float drift_term = interior ? DRIFT_COMPENSATION * compression : 0.0f;
        divergence -= drift_term;

        divergence *= OVERRELAXATION;

        float face_count = 0.0f;
        if (j > 0)
          face_count += 1.0f;
        if (j < width - 1)
          face_count += 1.0f;
        if (i > 0)
          face_count += 1.0f;
        if (i < height - 1)
          face_count += 1.0f;
        if (face_count <= 0.0f)
          continue;

        float correction = -divergence / face_count;

        // Accumulate a pressure-like potential for visualization.
        if (sim->pressure)
          sim->pressure[idx] += correction;

        if (j > 0)
          sim->grid.u_velocities[get_u_index(i, j, width, height)] -=
              correction;
        if (j < width - 1)
          sim->grid.u_velocities[get_u_index(i, j + 1, width, height)] +=
              correction;
        if (i > 0)
          sim->grid.v_velocities[get_v_index(i, j, width, height)] -=
              correction;
        if (i < height - 1)
          sim->grid.v_velocities[get_v_index(i + 1, j, width, height)] +=
              correction;
      }
    }
  }

  // Recenter the pressure-like field to remove arbitrary offset drift.
  float pressure_mean = 0.0f;
  int fluid_count = 0;
  if (sim->pressure) {
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        int idx = cell_index(i, j, width);
        if (sim->grid.cells[idx].type != FLUID)
          continue;
        pressure_mean += sim->pressure[idx];
        fluid_count++;
      }
    }

    if (fluid_count > 0) {
      pressure_mean /= (float)fluid_count;
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          int idx = cell_index(i, j, width);
          if (sim->grid.cells[idx].type != FLUID)
            continue;
          sim->pressure[idx] -= pressure_mean;
        }
      }
    }
  }

  // Keep debug/visualization fields in sync.
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int idx = cell_index(i, j, width);
      Cell *cell = &sim->grid.cells[idx];
      if (cell->type != FLUID) {
        cell->pressure = 0.0f;
        cell->divergence = 0.0f;
        continue;
      }

      float u_l = sim->grid.u_velocities[get_u_index(i, j, width, height)];
      float u_r = sim->grid.u_velocities[get_u_index(i, j + 1, width, height)];
      float v_b = sim->grid.v_velocities[get_v_index(i, j, width, height)];
      float v_t = sim->grid.v_velocities[get_v_index(i + 1, j, width, height)];

      float target_pressure = sim->pressure ? sim->pressure[idx] : 0.0f;
      // Temporal smoothing reduces pressure-color shimmer.
      cell->pressure = 0.9f * cell->pressure + 0.1f * target_pressure;
      cell->divergence = (u_r - u_l) + (v_t - v_b);
    }
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
  const int u_count = (sim->grid.width + 1) * sim->grid.height;
  const int v_count = sim->grid.width * (sim->grid.height + 1);

  particle_to_grid(sim);

  if (!sim->prev_u)
    sim->prev_u = malloc(sizeof(float) * u_count);
  if (!sim->prev_v)
    sim->prev_v = malloc(sizeof(float) * v_count);
  if (!sim->prev_u || !sim->prev_v)
    return;

  memcpy(sim->prev_u, sim->grid.u_velocities, sizeof(float) * u_count);
  memcpy(sim->prev_v, sim->grid.v_velocities, sizeof(float) * v_count);

  add_gravity_to_grid(sim, a, dt);
  make_incompressible(sim);
  grid_to_particle(sim, sim->prev_u, sim->prev_v, dt);
  resolve_particle_collisions(sim);
}
