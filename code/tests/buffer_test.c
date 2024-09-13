#include <stdio.h>

#include "buffer.h"
#include "module.h"

struct bfr_gcode_t bfr_gcode_cmds;


int main()
{
    int32_t rc;
    size_t buffer_size = 12;
    struct gcode_t item;

    bfr_init(&bfr_gcode_cmds, buffer_size);

    printf("size: %ld\n", sizeof(bfr_gcode_cmds));


    int i = 0;
    while (1) {
        if (i % 3 == 0) {
            item.cmd = G03;
        }
        else if (i % 2 == 0) {
            item.cmd = G02;
        }
        else {
            item.cmd = G01;
        }

        item.data = (struct gcode_data_t) {.x = -1*i, .y = i*i, .z = 5*i};
        printf("cmd: %d x: %f y: %f z: %f\n", item.cmd, item.data.x, item.data.y, item.data.z);

        rc = bfr_produce(&bfr_gcode_cmds, item);
        if (rc == BFR_ERR_FULL) {
            printf("[ERROR!] could not add item to buffer.\n");
            break;
        }
        i++;
    }

    printf("Added %d items.\n", i);
    printf("\n***************************\n\n");


    i = 0;
    while (1) {

        printf("next cmd: %d\n", bfr_gcode_cmds.info.next_cmd);

        rc = bfr_consume(&bfr_gcode_cmds, &item);
        if (rc == BFR_ERR_EMPTY) {
            printf("[ERROR!] could not take item from buffer.\n");
            break;
        }

        printf("cmd: %d x: %f y: %f z: %f\n\n", item.cmd, item.data.x, item.data.y, item.data.z);
        i++;
    }

    printf("Removed %d items.\n", i);

}