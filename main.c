#include "flip.h"
#include <math.h>
#include <raylib.h>
#include <raymath.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SCREENWIDTH 1200.0f
#define SCREENHEIGHT 800.0f

static int draw_button(Rectangle rect, const char *label, Color base,
                       Color hover) {
  Vector2 mouse = GetMousePosition();
  int is_hovered = CheckCollisionPointRec(mouse, rect);
  DrawRectangleRec(rect, is_hovered ? hover : base);
  DrawRectangleLinesEx(rect, 1.0f, WHITE);
  DrawText(label, (int)rect.x + 10, (int)rect.y + 6, 20, WHITE);
  return is_hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

static void draw_slider(const char *label, float *value, float min_v,
                        float max_v, Rectangle rect, float step) {
  Vector2 mouse = GetMousePosition();
  int active = IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
               CheckCollisionPointRec(mouse, rect);
  if (active) {
    float ratio = (mouse.x - rect.x) / rect.width;
    ratio = Clamp(ratio, 0.0f, 1.0f);
    float next = min_v + ratio * (max_v - min_v);
    if (step > 0.0f)
      next = roundf(next / step) * step;
    *value = Clamp(next, min_v, max_v);
  }

  float ratio = (*value - min_v) / (max_v - min_v);
  ratio = Clamp(ratio, 0.0f, 1.0f);
  DrawText(label, (int)rect.x, (int)rect.y - 20, 18, WHITE);
  DrawRectangleRec(rect, (Color){30, 30, 30, 220});
  Rectangle fill = {rect.x, rect.y, rect.width * ratio, rect.height};
  DrawRectangleRec(fill, (Color){70, 150, 255, 220});
  DrawRectangleLinesEx(rect, 1.0f, GRAY);

  char value_text[32];
  snprintf(value_text, sizeof(value_text), "%.3f", *value);
  DrawText(value_text, (int)(rect.x + rect.width + 10), (int)rect.y + 2, 16,
           WHITE);
}

static void draw_slider_int(const char *label, int *value, int min_v, int max_v,
                            Rectangle rect) {
  float slider_value = (float)(*value);
  draw_slider(label, &slider_value, (float)min_v, (float)max_v, rect, 1.0f);
  *value = (int)slider_value;
}

static void draw_checkbox(const char *label, bool *checked, Rectangle box) {
  Vector2 mouse = GetMousePosition();
  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
      CheckCollisionPointRec(mouse, box))
    *checked = !*checked;

  DrawRectangleRec(box, (Color){20, 20, 20, 220});
  DrawRectangleLinesEx(box, 1.0f, WHITE);
  if (*checked)
    DrawText("X", (int)box.x + 5, (int)box.y - 1, 18, GREEN);
  DrawText(label, (int)box.x + 26, (int)box.y - 1, 18, WHITE);
}

static void fill_initial_state(enum cell_type *fluid, int width, int height) {
  memset(fluid, AIR, sizeof(enum cell_type) * width * height);
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width / 2; j++)
      fluid[i * width + j] = FLUID;
  }
}

static Simulation reset_simulation(int width, int height,
                                   const SimParams *params) {
  enum cell_type *fluid =
      malloc(sizeof(enum cell_type) * (size_t)width * (size_t)height);
  fill_initial_state(fluid, width, height);
  Simulation sim = initiallise_simulation(width, height, fluid, params);
  free(fluid);
  return sim;
}

int main(int argc, char *argv[]) {
  InitWindow((int)SCREENWIDTH, (int)SCREENHEIGHT, "Fluid Sim");

  int offset = 820;
  int fps;
  char fps_str[16];

  const float fixed_dt = 1.0f / 60.0f;
  float accumulator = 0.0f;

  SimParams params = default_sim_params();
  int paused = 0;

  bool show_particles = true;
  bool show_pressure = true;
  bool show_velocity = true;
  bool show_grid = true;
  bool show_divergence = true;

  int sim_width = SIMWIDTH;
  int sim_height = SIMHEIGHT;

  Vector2 g = {0.0f, 9.8f};
  Simulation sim = reset_simulation(sim_width, sim_height, &params);
  float display_abs_pressure = 1.0f;

  while (!WindowShouldClose()) {
    float frame_dt = GetFrameTime();
    if (frame_dt > 0.033f)
      frame_dt = 0.033f;

    if (!paused) {
      accumulator += frame_dt;
      while (accumulator >= fixed_dt) {
        compute(&sim, g, fixed_dt, &params);
        accumulator -= fixed_dt;
      }
    }

    float cell_size_x = SCREENWIDTH / (float)sim.grid.width;
    float cell_size_y = SCREENHEIGHT / (float)sim.grid.height;
    float cell_size = fminf(cell_size_x, cell_size_y);

    BeginDrawing();
    ClearBackground(BLACK);

    if (show_particles) {
      for (int i = 0; i < sim.particles.size; i++) {
        int x = (int)(sim.particles.data[i].position.x * cell_size);
        int y = (int)(sim.particles.data[i].position.y * cell_size);
        float radius = params.particle_radius * cell_size;
        DrawCircle(x, y, radius, WHITE);
      }
    }

    if (show_pressure) {
      float max_abs_pressure = 0.0f;
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int idx = i * sim.grid.width + j;
          if (sim.grid.cells[idx].type != FLUID)
            continue;

          float abs_p = fabsf(sim.grid.cells[idx].pressure);
          if (abs_p > max_abs_pressure)
            max_abs_pressure = abs_p;
        }
      }

      if (max_abs_pressure < 0.001f)
        max_abs_pressure = 0.001f;

      display_abs_pressure = max_abs_pressure;
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int idx = i * sim.grid.width + j;
          if (sim.grid.cells[idx].type != FLUID)
            continue;

          float p = sim.grid.cells[idx].pressure;
          float t = 0.5f + 0.5f * (p / display_abs_pressure);
          t = Clamp(t, 0.0f, 1.0f);

          Color pressure_color = {(unsigned char)(40 + 180 * t), 40,
                                  (unsigned char)(220 - 180 * t), 130};
          DrawRectangle((int)(j * cell_size), (int)(i * cell_size),
                        (int)cell_size + 1, (int)cell_size + 1, pressure_color);
        }
      }
    }

    if (show_grid) {
      for (int i = 0; i <= sim.grid.height; i++) {
        DrawLine(0, (int)(i * cell_size), (int)(sim.grid.width * cell_size),
                 (int)(i * cell_size), GRAY);
      }
      for (int i = 0; i <= sim.grid.width; i++) {
        DrawLine((int)(i * cell_size), 0, (int)(i * cell_size),
                 (int)(sim.grid.height * cell_size), GRAY);
      }
    }

    if (show_velocity) {
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int u_x1 = (int)(j * cell_size);
          int u_x2 = (int)(u_x1 + sim.grid.u_velocities[get_u_index(
                                      i, j, sim.grid.width, sim.grid.height)] *
                                      (cell_size / 4));
          int u_y = (int)((i * cell_size) + cell_size / 2);

          int v_x = (int)((j * cell_size) + cell_size / 2);
          int v_y1 = (int)(i * cell_size);
          int v_y2 = (int)(v_y1 + sim.grid.v_velocities[get_v_index(
                                      i, j, sim.grid.width, sim.grid.height)] *
                                      (cell_size / 4));
          DrawLine(u_x1, u_y, u_x2, u_y, RED);
          DrawLine(v_x, v_y1, v_x, v_y2, RED);
        }
      }
    }

    if (show_divergence) {
      char d[8];
      int font_size = (int)fmaxf(10.0f, cell_size / 4.0f);
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int idx = i * sim.grid.width + j;
          snprintf(d, sizeof(d), "%.1f", sim.grid.cells[idx].divergence);
          DrawText(d, (int)(j * cell_size + cell_size / 3),
                   (int)(i * cell_size + cell_size / 3), font_size, WHITE);
        }
      }
    }

    Rectangle panel = {offset + 10, 50, 320, 690};
    DrawRectangleRec(panel, (Color){0, 0, 0, 180});
    DrawRectangleLinesEx(panel, 1.0f, GRAY);

    if (draw_button((Rectangle){offset + 20, 60, 120, 35},
                    paused ? "Play" : "Pause", (Color){40, 120, 60, 200},
                    (Color){60, 160, 90, 220})) {
      paused = !paused;
    }

    int requested_reset = 0;
    if (draw_button((Rectangle){offset + 150, 60, 120, 35}, "Reset",
                    (Color){120, 60, 40, 200}, (Color){160, 80, 60, 220})) {
      requested_reset = 1;
    }

    draw_checkbox("Draw particles", &show_particles,
                  (Rectangle){offset + 20, 110, 18, 18});
    draw_checkbox("Draw pressure", &show_pressure,
                  (Rectangle){offset + 20, 140, 18, 18});
    draw_checkbox("Draw velocity", &show_velocity,
                  (Rectangle){offset + 20, 170, 18, 18});
    draw_checkbox("Draw divergence", &show_divergence,
                  (Rectangle){offset + 20, 200, 18, 18});

    draw_slider_int("Simulation width", &sim_width, 4, 40,
                    (Rectangle){offset + 20, 250, 220, 16});
    draw_slider_int("Simulation height", &sim_height, 4, 40,
                    (Rectangle){offset + 20, 290, 220, 16});
    draw_slider("Gravity X", &g.x, -30.0f, 30.0f,
                (Rectangle){offset + 20, 340, 220, 16}, 0.1f);
    draw_slider("Gravity Y", &g.y, -30.0f, 30.0f,
                (Rectangle){offset + 20, 380, 220, 16}, 0.1f);

    draw_slider("Flip blend", &params.flip_blend, 0.0f, 1.0f,
                (Rectangle){offset + 20, 430, 220, 16}, 0.01f);
    draw_slider_int("Gauss-Seidel iterations", &params.gauss_iters, 1, 32,
                    (Rectangle){offset + 20, 470, 220, 16});
    draw_slider("Boundary damping", &params.boundary_damping, -1.0f, 1.0f,
                (Rectangle){offset + 20, 510, 220, 16}, 0.01f);
    draw_slider_int("Collision iterations", &params.particle_collision_iters, 1,
                    10, (Rectangle){offset + 20, 550, 220, 16});
    draw_slider("Particle radius", &params.particle_radius, 0.05f, 0.49f,
                (Rectangle){offset + 20, 590, 220, 16}, 0.01f);
    draw_slider("Drift compensation", &params.drift_compensation, 0.0f, 5.0f,
                (Rectangle){offset + 20, 630, 220, 16}, 0.01f);
    draw_slider("Overrelaxation", &params.overrelaxation, 0.1f, 2.5f,
                (Rectangle){offset + 20, 670, 220, 16}, 0.01f);
    draw_slider_int("Particles / cell", &params.particles_per_cell, 1, 12,
                    (Rectangle){offset + 20, 710, 220, 16});

    DrawText("Reset applies size and particles/cell.", offset + 20, 750, 16,
             LIGHTGRAY);

    if (requested_reset) {
      destroy_sim(&sim);
      sim = reset_simulation(sim_width, sim_height, &params);
      accumulator = 0.0f;
    }

    fps = GetFPS();
    snprintf(fps_str, sizeof(fps_str), "FPS: %04d", fps);
    DrawText(fps_str, offset + 10, 10, 30, WHITE);
    EndDrawing();
  }

  destroy_sim(&sim);
  CloseWindow();
  return 1;
}
