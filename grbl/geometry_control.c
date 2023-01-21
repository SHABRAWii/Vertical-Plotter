/*
  geometry_control.c - change geometry system from coordinate system to our system.

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

/*
    This Geometry type needs to know always the current position of the machine and where it is in real life.

    So It will be better to use the G91 command to make the machine work in relative coordinates.
    After the G91 command any movements or coordinates specified will be relative to the current position of the machine, rather than being absolute coordinates. 
      For example, if the machine is currently at position (x=5, y=5), and the G-code specifies G91 G0 X10 Y10, the machine will move to position (x=15, y=15) rather than (x=10, y=10).
    
    => We always translate the origin of the machine to the HOME_Point (0,0) in the real world.

                                                                                      (-y)
                                                                                        |
    M1->[-HOME_X, HOME_Y] (+y)                 [HOME_X,HOME_Y]<-M2    (-x)--------------+---------------(+x)    [WIDTH,0]<-M2
                            |                                                          /|
                            |                                        =>      [0, 0]<-M1 |
                            |  HOME_Point->[0, 0]                    =>                 |  
                            | /                                      =>                 | 
        (-x)----------------+---------------(+x)                     =>                 |           _x_ ->HOME_Point->[HOME_X, -HOME_Y] 
                            |                                        =>                 | 
                            |                                        =>                 |
                            |                                        =>                 |
                          (-y)                                       =>                (+y)
                    (Normal Coordinates)                                           (New Coordinates)

                If you have a point you want to translate it in general it will be like this:
                Point(x, y) => Point(x - x0, y - y0) :: x0 and y0 are the translation values.
    
              So here If we want to translate from Normal Coordinates to New Coordinates we will use the following:
                              Point(x, y) => Point(x + HOME_X, -(y - HOME_Y))
              And IF we want to translate from New Coordinates to Normal Coordinates we will use the following:
                              Point(x, y) => Point(x - HOME_X, -(y) + HOME_Y)
  */


struct {
  long double x;
  long double y;
} cur_position = {0, 0};

void Convert_Geometry(int32_t *planner_position, float *target, plan_block_t *block, float *unit_vec, int32_t *target_steps)
{
  { // Correcting Error of the feedback of the system
    sys.position[X_AXIS] = planner_position[X_AXIS];
    sys.position[Y_AXIS] = planner_position[Y_AXIS];
  }
  /*
    $(planner_position) refers to   $(pl.position)   :: current position of the machine   in steps
    $(target)           refers to   $(target)        :: target position of the machine    in mm
    $(block)            refers to   $(block)         :: block of the machine
    $(unit_vec)         refers to   $(unit_vec)      :: unit vector of the machine
    $(target_steps)     refers to   $(target_steps)  :: target steps of the machine
  */
  
  // Convert current position from steps to mm
  point from = {planner_position[X_AXIS] / settings.steps_per_mm[X_AXIS], planner_position[Y_AXIS] / settings.steps_per_mm[Y_AXIS]};
  point to = {target[X_AXIS], target[Y_AXIS]};
  TO_NEW_GEOMETRY(from); // Convert to New Coordinates
  TO_NEW_GEOMETRY(to);   // Convert to New Coordinates
  
  double from_M[3] = {
      hypot(from.x, from.y),                 // Distance form M1 to point that we move from
      hypot(MACHINE_WIDTH - from.x, from.y), // Distance form M2 to point that we move from
      0
  };
  double to_M[3] = {
      hypot(to.x, to.y),                 // Distance form M1 to point that we move to
      hypot(MACHINE_WIDTH - to.x, to.y), // Distance form M2 to point that we move from
      0
  };
  point _NEW = {0, 0};
  { // point with least error correction 
    /*
      Youssef: Assume that:
        1. We measure the home point correctly
        2. Target point is not reachable by the machine
    */
    long double dM1_p[2] = {
        floor((to_M[X_AXIS] - from_M[X_AXIS]) * settings.steps_per_mm[X_AXIS]) / settings.steps_per_mm[X_AXIS],
        ceil((to_M[X_AXIS] - from_M[X_AXIS]) * settings.steps_per_mm[X_AXIS]) / settings.steps_per_mm[X_AXIS]
    };
    long double dM2_p[2] = {
        floor((to_M[Y_AXIS] - from_M[Y_AXIS]) * settings.steps_per_mm[Y_AXIS]) / settings.steps_per_mm[Y_AXIS],
        ceil((to_M[Y_AXIS] - from_M[Y_AXIS]) * settings.steps_per_mm[Y_AXIS]) / settings.steps_per_mm[Y_AXIS]
    };
    long double ERROR = 100000;
    for(int i = 0; i < 2; i++) {     // Loop over all possible values of dM1
        for(int j = 0; j < 2; j++) { // Loop over all possible values of dM2
            double dM1 = dM1_p[i];
            double dM2 = dM2_p[j];
            double _to_M[2] = { // possible Distance form M1 to point that we move to
                from_M[X_AXIS] + dM1,
                from_M[Y_AXIS] + dM2,
            };
            double s = SEMI_PERIMETER(fabs(_to_M[X_AXIS]), fabs(_to_M[Y_AXIS]), MACHINE_WIDTH);
            double area = AREA(s, fabs(_to_M[X_AXIS]), fabs(_to_M[Y_AXIS]), MACHINE_WIDTH);
            double _y = 2.0 * area / (double)(MACHINE_WIDTH);
            double _x = sqrt(_to_M[X_AXIS] * _to_M[X_AXIS] - _y * _y);
            long double _ERROR = hypot(fabs(_x - to.x), fabs(_y - to.y));
            if(_ERROR < ERROR) {
                ERROR = _ERROR;
                to_M[X_AXIS] = _to_M[X_AXIS];
                to_M[Y_AXIS] = _to_M[Y_AXIS];
                _NEW.x = _x;
                _NEW.y = _y;
            }
        }
    }
  }
  // Calculated needed steps for each motor
  float delta_mm;
  block->millimeters = 0;
  TO_OLD_GEOMETRY(_NEW); // Convert to Old Coordinates
  for(int idx=0; idx<N_AXIS; idx++) {
    target_steps[idx] = (_NEW.x * (idx == X_AXIS) + _NEW.y * (idx == Y_AXIS)) * settings.steps_per_mm[idx]; // move target to new point with least error

    delta_mm = (to_M[idx] - from_M[idx]);

    block->steps[idx] = labs(delta_mm * settings.steps_per_mm[idx]); // Calculate difference in strings length and convert to steps
    block->step_event_count = max(block->step_event_count, block->steps[idx]); 

    if (delta_mm < 0 ) { block->direction_bits |= get_direction_pin_mask(idx); }

    unit_vec[idx] = delta_mm;     // Calculate unit vector

    block->millimeters += delta_mm * delta_mm; // Sum all axis vectors
  }
  
  block->millimeters = sqrt(block->millimeters); // Complete millimeters calculation with sqrt()
  // recalculate new position due to geometry cababilities
  {
    // float _to_M[3] = {
    //   from_M[X_AXIS] + (unit_vec[X_AXIS] > 0 ? 1 : -1)*(int32_t)(block->steps[X_AXIS]) / settings.steps_per_mm[X_AXIS],
    //   from_M[Y_AXIS] + (unit_vec[Y_AXIS] > 0 ? 1 : -1)*(int32_t)(block->steps[Y_AXIS]) / settings.steps_per_mm[Y_AXIS],
    //   0
    // };
    // double s = SEMI_PERIMETER(fabs(_to_M[X_AXIS]), fabs(_to_M[Y_AXIS]), MACHINE_WIDTH);
    // double area = AREA(s, fabs(_to_M[X_AXIS]), fabs(_to_M[Y_AXIS]), MACHINE_WIDTH);
    // double _y = 2.0 * area / MACHINE_WIDTH;
    // double _x = sqrt(_to_M[X_AXIS] * _to_M[X_AXIS] - _y * _y);

  }
  return;
}
void Geometry_system_position(uint8_t axis, bool direction, float *sys_position)
{
  point from = {sys_position[X_AXIS] / settings.steps_per_mm[X_AXIS], sys_position[Y_AXIS] / settings.steps_per_mm[Y_AXIS]};
  TO_NEW_GEOMETRY(from);
  float from_M[3] = {
      hypotf(from.x, from.y),                 // Distance form M1 to point that we move from
      hypotf(MACHINE_WIDTH - from.x, from.y), // Distance form M2 to point that we move from
      0
  };
  float to_M[3] = {
      from_M[X_AXIS] + (direction ? 1 : -1) * (axis == X_AXIS) / settings.steps_per_mm[X_AXIS],                 // Distance form M1 to point that we move to
      from_M[Y_AXIS] + (direction ? 1 : -1) * (axis == Y_AXIS) / settings.steps_per_mm[Y_AXIS], // Distance form M2 to point that we move from
      0
  };
  float to_Semi_Perimeter = SEMI_PERIMETER(fabs(to_M[X_AXIS]), fabs(to_M[Y_AXIS]), MACHINE_WIDTH);
  float to_Area = AREA(to_Semi_Perimeter, fabs(to_M[X_AXIS]), fabs(to_M[Y_AXIS]), MACHINE_WIDTH);
  point to = {0, 0};
  to.y = (to_Area * 2.0) / MACHINE_WIDTH; // Translate from home position
  to.x = sqrt(to_M[X_AXIS] * to_M[X_AXIS] - to.y * to.y); // Translate from home position
  TO_OLD_GEOMETRY(from);
  TO_OLD_GEOMETRY(to);
  if(axis == X_AXIS){
    sys_position[X_AXIS] += (to.x - from.x) * settings.steps_per_mm[X_AXIS];
    sys_position[Y_AXIS] += (to.y - from.y) * settings.steps_per_mm[Y_AXIS];
  }else{
    sys_position[X_AXIS] += (to.x - from.x) * settings.steps_per_mm[X_AXIS];
    sys_position[Y_AXIS] += (to.y - from.y) * settings.steps_per_mm[Y_AXIS];
  }
}


#endif