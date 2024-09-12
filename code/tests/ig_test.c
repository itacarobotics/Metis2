#include <stdio.h>

#include "trajectory_generator.h"
#include "inverse_geometry.h"


int main()
{
    struct gcode_t pos_home = {.cmd = G01, .data = {.x = -40, .y = 30, .z = -100, .i = 0, .j = 0, .k = 0, .t = 0}};
    struct gcode_t pos_goal   = {.cmd = G01, .data = {.x = 0, .y = 0, .z = -180, .i = 0, .j = 0, .k = 0, .t = 0}};
    struct gcode_t pos;
    enum rc_status_t rc;

    tg_init(pos_home);

    rc = tg_set_new_trajectory(pos_goal);
    if (rc != VALID_TRAJECTORY) {
        printf("[ERROR!] Not a valid trajectory.\n");
        return -1;
    }

    while (1) {
        rc = tg_get_via_point(&pos);
        if (rc == END_OF_TRAJECTORY) {
            printf("[INFO] End of trajectory\n\n");
            break;
        }

        rc = inverse_geometry(&pos);
        if (rc != VALID_IG_SOLUTION) {
            printf("[ERROR!] '%d'Could not find a solution.\n", rc);
            break;
        }

        printf("x:  % 5.3f, y:  % 5.3f, z:  % 5.3f, k:  % 5.3f, t:  % 5.3f\n", pos.data.x, pos.data.y, pos.data.z, pos.data.k, pos.data.t);
        printf("q1: % 5.3f, q2: % 5.3f, q3: % 5.3f, q4: % 5.3f, t: % 5.3f\n\n", pos.data.q1, pos.data.q2, pos.data.q3, pos.data.q4, pos.data.t);
    }
}