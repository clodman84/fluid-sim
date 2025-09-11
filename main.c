#include "flip.h"
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <string.h>

#define SCREENWIDTH 800.0
#define SCREENHEIGHT 800.0
#define SHOWPARTICLE 1
#define SHOWGRID 1
#define SHADEGRID 0
#define SHOWVELOCITY 1

int main(int argc, char *argv[]) {
  InitWindow(SCREENWIDTH, SCREENHEIGHT, "Fluid Sim");
  int fps;
  char fps_str[10];
  float dt;

  Vector2 g = {0, 9.8};
  Rectangle rect = {200, 100, CELL_SIZE / 20, CELL_SIZE / 20};
  enum cell_type fluid[SIMWIDTH * SIMHEIGHT];
  // memset(fluid, AIR, sizeof(fluid));
  // fluid[10] = FLUID;
  // fluid[20] = FLUID;
  for (int i = 0; i < SIMHEIGHT * SIMWIDTH; i++) {
    fluid[i] = FLUID;
  }
  Simulation sim = initiallise_simulation(SIMWIDTH, SIMHEIGHT, fluid);

  while (!WindowShouldClose()) {
    dt = GetFrameTime();
    BeginDrawing();
    ClearBackground(BLACK);
    // g.x += dt;
    compute(&sim, g, dt);
    // printf("G: {%f, %f}\n", g.x, g.y);
    if (SHOWPARTICLE) {
      for (int i = 0; i < sim.particles.size; i++) {
        rect.x = sim.particles.data[i].position.x * CELL_SIZE;
        rect.y = sim.particles.data[i].position.y * CELL_SIZE;
        DrawRectangleRec(rect, WHITE);
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
    if (SHADEGRID) {
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int idx = i * sim.grid.width + j;
          if (sim.grid.cells[idx].type == FLUID)
            DrawRectangle(j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE,
                          LIGHTGRAY);
          ;
        }
      }
    }
    if (SHOWVELOCITY) {
      for (int i = 0; i < sim.grid.height; i++) {
        for (int j = 0; j < sim.grid.width; j++) {
          int u_x1 = j * CELL_SIZE;
          int u_x2 =
              u_x1 +
              sim.grid.u_velocities[get_u_index(i, j, SIMWIDTH, SIMHEIGHT)];
          int u_y = (i * CELL_SIZE) + CELL_SIZE / 2;

          int v_x = (j * CELL_SIZE) + CELL_SIZE / 2;
          int v_y1 = i * CELL_SIZE;
          int v_y2 =
              v_y1 +
              sim.grid.v_velocities[get_v_index(i, j, SIMWIDTH, SIMHEIGHT)];
          DrawLine(u_x1, u_y, u_x2, u_y, RED);
          DrawLine(v_x, v_y1, v_x, v_y2, RED);
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
