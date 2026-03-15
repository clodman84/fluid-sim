#include "flip.h"
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <string.h>

#define SCREENWIDTH 800.0
#define SCREENHEIGHT 800.0
#define CELL_SIZE 100

#define SHOWPARTICLE 1
#define SHOWGRID 1
#define SHADEGRID 1
#define SHOWVELOCITY 1
#define SHOWDIVERGENCE 1

int main(int argc, char *argv[]) {
  InitWindow(SCREENWIDTH, SCREENHEIGHT, "Fluid Sim");
  // SetTargetFPS(60);

  int fps;
  char fps_str[10];

  const float fixed_dt = 1.0f / 60.0f;
  float accumulator = 0.0f;

  Vector2 g = {0, 9.8};
  enum cell_type fluid[SIMWIDTH * SIMHEIGHT];
  memset(fluid, AIR, sizeof(fluid));

  for (int i = 0; i < SIMHEIGHT; i++) {
    for (int j = 0; j < SIMWIDTH / 2; j++) {
      fluid[i * SIMWIDTH + j] = FLUID;
    }
  }

  Simulation sim = initiallise_simulation(SIMWIDTH, SIMHEIGHT, fluid);
  float display_abs_pressure = 1.0f;
  while (!WindowShouldClose()) {
    float frame_dt = GetFrameTime();
    if (frame_dt > 0.033f)
      frame_dt = 0.033f;

    accumulator += frame_dt;
    while (accumulator >= fixed_dt) {
      compute(&sim, g, fixed_dt);
      accumulator -= fixed_dt;
    }

    // compute(&sim, g, frame_dt);
    BeginDrawing();
    ClearBackground(BLACK);

    if (SHOWPARTICLE) {
      for (int i = 0; i < sim.particles.size; i++) {
        int x = sim.particles.data[i].position.x * CELL_SIZE;
        int y = sim.particles.data[i].position.y * CELL_SIZE;
        float radius = PARTICLE_RADIUS * CELL_SIZE;
        DrawCircle(x, y, radius, WHITE);
      }
    }

    if (SHADEGRID) {
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

      // Exponential smoothing avoids frame-to-frame flicker.
      // display_abs_pressure =
      //     display_abs_pressure * 0.95f + max_abs_pressure * 0.05f;
      // if (display_abs_pressure < 0.001f)
      //   display_abs_pressure = 0.001f;

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
          DrawRectangle(j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE,
                        pressure_color);
        }
      }
    }

    if (SHOWGRID) {
      for (int i = 0; i <= SIMHEIGHT; i++) {
        DrawLine(0, i * CELL_SIZE, SIMWIDTH * CELL_SIZE, i * CELL_SIZE, GRAY);
      }
      for (int i = 0; i <= SIMWIDTH; i++) {
        DrawLine(i * CELL_SIZE, 0, i * CELL_SIZE, SIMHEIGHT * CELL_SIZE, GRAY);
      }
    }

    if (SHOWVELOCITY) {
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int u_x1 = j * CELL_SIZE;
          int u_x2 =
              u_x1 +
              sim.grid.u_velocities[get_u_index(i, j, SIMWIDTH, SIMHEIGHT)] *
                  (CELL_SIZE / 4);
          int u_y = (i * CELL_SIZE) + CELL_SIZE / 2;

          int v_x = (j * CELL_SIZE) + CELL_SIZE / 2;
          int v_y1 = i * CELL_SIZE;
          int v_y2 =
              v_y1 +
              sim.grid.v_velocities[get_v_index(i, j, SIMWIDTH, SIMHEIGHT)] *
                  (CELL_SIZE / 4);
          DrawLine(u_x1, u_y, u_x2, u_y, RED);
          DrawLine(v_x, v_y1, v_x, v_y2, RED);
        }
      }
    }

    if (SHOWDIVERGENCE) {
      char d[5];
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int idx = i * sim.grid.width + j;
          sprintf(d, "%.1f", sim.grid.cells[idx].divergence);
          DrawText(d, j * CELL_SIZE + CELL_SIZE / 3,
                   i * CELL_SIZE + CELL_SIZE / 3, CELL_SIZE / 3, WHITE);
        }
      }
    }

    fps = GetFPS();
    sprintf(fps_str, "FPS: %04d", fps);
    DrawText(fps_str, 10, 10, 30, WHITE);
    EndDrawing();
  }

  destroy_sim(&sim);
  CloseWindow();
  return 1;
}
