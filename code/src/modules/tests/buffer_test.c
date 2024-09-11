#include <stdio.h>

#include "../include/buffer.h"

struct buffer_gcode_t buffer_gcode_cmds;


int main()
{
    uint16_t rc;
    uint32_t buffer_size = 12;
    struct gcode_t item;

    buffer_init(&buffer_gcode_cmds, buffer_size);

    printf("size: %ld\n", sizeof(buffer_gcode_cmds));


    int i = 0;
    while (1) {
        item.cmd = G03;
        item.data = (struct gcode_data_t) {.x = -1*i, .y = i*i, .z = 5*i};
        printf("cmd: %d x: %f y: %f z: %f\n", item.cmd, item.data.x, item.data.y, item.data.z);

        printf("buffer status: %d\n", buffer_is_full(&buffer_gcode_cmds));
        rc = buffer_produce(&buffer_gcode_cmds, item);
        if (rc) {
            printf("[ERROR!] could not add item to buffer.\n");
            break;
        }
        i++;
    }

    printf("buffer status: %d\n", buffer_is_full(&buffer_gcode_cmds));
    printf("Added %d items.\n", i);

    printf("\n\n***************************\n\n");

    i = 0;
    while (1) {
        printf("buffer status: %d\n", buffer_is_empty(&buffer_gcode_cmds));
        rc = buffer_consume(&buffer_gcode_cmds, &item);
        if (rc) {
            printf("[ERROR!] could not take item from buffer.\n");
            break;
        }

        printf("cmd: %d x: %f y: %f z: %f\n", item.cmd, item.data.x, item.data.y, item.data.z);
        i++;
    }

    printf("buffer status: %d\n", buffer_is_empty(&buffer_gcode_cmds));
    printf("Removed %d items.\n", i);

}