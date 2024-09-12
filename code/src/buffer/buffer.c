/*
 * @brief Example template.
 *
 * MIT License
 * 
 * Copyright (c) 2024 Federico Osti
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */



#include "buffer.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////


/**
 *  @brief Init buffer.
 *  @param  *buffer Pointer to buffer to be initialized.
 */
void bfr_init(struct bfr_gcode_t *buffer, uint32_t buffer_size)
{
    buffer->size = buffer_size;
    buffer->data = malloc(buffer_size * sizeof(struct gcode_t));

    buffer->status      = EMPTY;
    buffer->length      = 0;
    buffer->head_idx    = 0;
    buffer->tail_idx    = 0;
    buffer->mutex       = 1;
    return;
}


enum bfr_status_t bfr_produce( struct bfr_gcode_t *buffer, struct gcode_t item)
{
    if (buffer->length == buffer->size) {
        buffer->status = FULL;
        return FULL;
    }
    if (!buffer->mutex) {
        buffer->status = BUSY;
        return BUSY;   // the consumer is taking an item
    }

    buffer->mutex = 0;      // lock process

    buffer->data[buffer->head_idx] = item;
    buffer->head_idx = (buffer->head_idx + 1) % buffer->size;
    buffer->length += 1;

    buffer->mutex = 1;       // unlock process

    return VALID;        // succesfully added new item to buffer
}


enum bfr_status_t bfr_consume(struct bfr_gcode_t *buffer, struct gcode_t *item)
{
    if (buffer->length == 0) {
        buffer->status = EMPTY;
        return EMPTY;
    }
    if (!buffer->mutex) {
        buffer->status = BUSY;
        return BUSY;   // the producer is adding an item
    }

    buffer->mutex = 0;      // lock process

    *item = buffer->data[buffer->tail_idx];
    buffer->tail_idx = (buffer->tail_idx + 1) % buffer->size;
    buffer->length -= 1;

    buffer->mutex = 1;       // unlock process

    return VALID;        // succesfully taken item from buffer
}


enum bfr_status_t bfr_get_status(struct bfr_gcode_t *buffer)
{
    return buffer->status;
}

void bfr_reset(struct bfr_gcode_t *buffer)
{
    buffer->length      = 0;
    buffer->head_idx    = 0;
    buffer->tail_idx    = 0;
    return;
}


////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////
