#include "flip.h"
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <string.h>

#define SCREENWIDTH 800.0
#define SCREENHEIGHT 800.0
#define SHOWGRID 1

int main(int argc, char *argv[]) {
  InitWindow(SCREENWIDTH, SCREENHEIGHT, "Fluid Sim");
  int fps;
  char fps_str[10];
  float dt;

  Vector2 g = {0, 9.8};
  Rectangle rect = {200, 100, 10, 10};
  enum cell_type fluid[SIMWIDTH * SIMHEIGHT];
  memset(fluid, AIR, sizeof(fluid));
  fluid[0] = FLUID;
  fluid[10] = FLUID;
  fluid[20] = FLUID;
  // for (int i = 0; i < SIMHEIGHT * SIMWIDTH; i++) {
  //   fluid[i] = FLUID;
  // }
  Simulation sim = initiallise_simulation(SIMWIDTH, SIMHEIGHT, fluid);

  while (!WindowShouldClose()) {
    dt = GetFrameTime();
    BeginDrawing();
    ClearBackground(BLACK);

    if (SHOWGRID) {
      for (int i = 0; i <= SIMHEIGHT; i++) {
        DrawLine(0, i * CELL_SIZE, SIMWIDTH * CELL_SIZE, i * CELL_SIZE, GRAY);
      }
      for (int i = 0; i <= SIMWIDTH; i++) {
        DrawLine(i * CELL_SIZE, 0, i * CELL_SIZE, SIMHEIGHT * CELL_SIZE, GRAY);
      }
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
    // g.x += dt;
    compute(&sim, g, dt);
    // printf("G: {%f, %f}\n", g.x, g.y);
    for (int i = 0; i < sim.particles.size; i++) {
      rect.x = sim.particles.data[i].position.x * CELL_SIZE;
      rect.y = sim.particles.data[i].position.y * CELL_SIZE;
      DrawRectangleRec(rect, WHITE);
    }

    fps = GetFPS();
    sprintf(fps_str, "FPS: %04d", fps);
    DrawText(fps_str, 10, 10, 30, WHITE);
    EndDrawing();
  }
  CloseWindow();
  return 1;
}
