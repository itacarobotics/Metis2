#include <stdio.h>

#include "trajectory_generator.h"


int main()
{
    struct gcode_t pos_start =  {.cmd = G01, .data = {.x = 0, .y = 0, .z = 0, .i = 0, .j = 0, .k = 0, .t = 0}};
    struct gcode_t pos_end =    {.cmd = G01, .data = {.x = -20, .y = 50, .z = 100, .i = 0, .j = 0, .k = 0, .t = 2.3}};
    struct gcode_t pos = pos_start;
    enum rc_status_t rc;

    tg_init(pos_start);

    rc = tg_set_new_trajectory(pos_end);
    if (rc != VALID_TRAJECTORY) {
        printf("[ERROR!] Not a valid trajectory.");
        return -1;
    }

    while (1) {
        rc = tg_get_via_point(&pos);
        if (rc == END_OF_TRAJECTORY) {
            break;
        }

        printf("x: %f, y: %f, z: %f, k: %f, t: %f\n", pos.data.x, pos.data.y, pos.data.z, pos.data.k, pos.data.t);
    }
}