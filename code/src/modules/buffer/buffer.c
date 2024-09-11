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



#include "../include/buffer.h"

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
void buffer_init(struct buffer_gcode_t *buffer, uint32_t buffer_size)
{
    buffer->size = buffer_size;
    buffer->data = malloc(buffer_size * sizeof(struct gcode_t));

    buffer->length      = 0;
    buffer->head_idx    = 0;
    buffer->tail_idx    = 0;
    buffer->mutex       = 1;
    return;
}


enum buffer_status_t buffer_produce( struct buffer_gcode_t *buffer, struct gcode_t item)
{
    if (buffer->length == buffer->size) {
        return -1;   // the buffer is full
    }
    if (!buffer->mutex) {
        return -1;   // the consumer is taking an item
    }

    buffer->mutex = 0;      // lock process

    buffer->data[buffer->head_idx] = item;
    buffer->head_idx = (buffer->head_idx + 1) % buffer->size;
    buffer->length += 1;

    buffer->mutex = 1;       // unlock process

    return 0;        // succesfully added new item to buffer
}


enum buffer_status_t buffer_consume(struct buffer_gcode_t *buffer, struct gcode_t *item)
{
    if (buffer->length == 0) {
        return -1;   // the buffer is empty
    }
    if (!buffer->mutex) {
        return -1;   // the producer is adding an item
    }

    buffer->mutex = 0;      // lock process

    *item = buffer->data[buffer->tail_idx];
    buffer->tail_idx = (buffer->tail_idx + 1) % buffer->size;
    buffer->length -= 1;

    buffer->mutex = 1;       // unlock process

    return 0;        // succesfully taken item from buffer
}


enum buffer_status_t buffer_is_full(struct buffer_gcode_t *buffer)
{
    if (buffer->length == buffer->size) {
        return 1;
    }
    return 0;
}


enum buffer_status_t buffer_is_empty(struct buffer_gcode_t *buffer)
{
    if (buffer->length == 0) {
        return 1;
    }
    return 0;
}


void buffer_reset(struct buffer_gcode_t *buffer)
{
    buffer->length      = 0;
    buffer->head_idx    = 0;
    buffer->tail_idx    = 0;
    return;
}


////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////
