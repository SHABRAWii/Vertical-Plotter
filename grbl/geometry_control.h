/*
  geometry_control.h - change geometry system from coordinate system to our system.

  Copyright (c) 2023 Youssef El Shabrawii

  Vertical Plotter is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Vertical Plotter is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "grbl.h"
#ifdef CPU_MAP_VERTICAL_PLOTTER

void Convert_Geometry(int32_t *from, float *to, plan_block_t *block, float *unit_vec, int32_t *target_steps);
void Geometry_system_position(uint8_t axis, bool direction, float *sys_position);

typedef struct {
    double x, y;
}point;
#define TO_NEW_GEOMETRY(point) point.x=point.x+settings.homing_x;point.y=-(point.y-settings.homing_y);
#define TO_OLD_GEOMETRY(point) point.x=point.x-settings.homing_x; point.y=-(point.y)+settings.homing_y;
#define SEMI_PERIMETER(a,b,c)(0.5*(a+b+c))
#define AREA(s,a,b,c)(sqrt(s*(s-a)*(s-b)*(s-c)))

#endif