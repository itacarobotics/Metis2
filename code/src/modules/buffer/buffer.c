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

static int32_t bfr_is_empty(bfr_gcode_t *buffer);
static int32_t bfr_is_full(bfr_gcode_t *buffer);

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
int32_t bfr_init(bfr_gcode_t *buffer, uint32_t buffer_size)
{
    buffer->size = buffer_size;
    buffer->data = malloc(buffer_size * sizeof(gcode_t));

    // buffer->status     = BFR_EMPTY;
    buffer->length     = 0;
    buffer->head_idx   = 0;
    buffer->tail_idx   = 0;
    buffer->mutex      = 1;
    return MOD_RET_OK;
}


/**
 * @brief Adds an item to the buffer
 * @param *buffer Pointer to buffer.
 * @param item Item to be added to the buffer.
 * @return ***
 * @note Function is concurrency mutex protected.
 */
int32_t bfr_produce(bfr_gcode_t *buffer, gcode_t item)
{
    if (bfr_is_full(buffer)) {
        return BFR_ERR_FULL;
    }
    if (!buffer->mutex) {
        return BFR_ERR_BUSY;   // the consumer is taking an item
    }

    buffer->mutex = 0;      // lock process

    buffer->data[buffer->head_idx] = item;
    buffer->head_idx = (buffer->head_idx + 1) % buffer->size;
    buffer->length += 1;

    buffer->mutex = 1;       // unlock process

    return MOD_RET_OK;        // succesfully added new item to buffer
}


/**
 * @brief Returns pointer to the next item in buffer to be consumed.
 * @param *buffer Pointer to buffer.
 * @param *item Pointer where to save the item.
 * @return ***
 * @note Function is concurrency mutex protected.
 */
int32_t bfr_consume(bfr_gcode_t *buffer, gcode_t *item)
{
    if (bfr_is_empty(buffer)) {
        return BFR_ERR_EMPTY;
    }

    if (!buffer->mutex) {
        return BFR_ERR_BUSY;   // the producer is adding an item
    }

    buffer->mutex = 0;      // lock process

    // consume item
    *item = buffer->data[buffer->tail_idx];
    buffer->tail_idx = (buffer->tail_idx + 1) % buffer->size;
    buffer->length -= 1;

    buffer->mutex = 1;       // unlock process

    return MOD_RET_OK;        // succesfully taken item from buffer
}

/**
 * @brief Reset buffer.
 * @param *buffer Pointer to buffer.
 * @note Does not reallocate memory.
 */
int32_t bfr_reset(bfr_gcode_t *buffer)
{
    buffer->length      = 0;
    buffer->head_idx    = 0;
    buffer->tail_idx    = 0;
    return MOD_RET_OK;
}


////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks wether the buffer is empty.
 * @param *buffer Pointer to buffer.
 * @return 1 if true, 0 if false.
 */
static int32_t bfr_is_empty(bfr_gcode_t *buffer)
{
    if (buffer->length == 0) {
        return 1;   // true
    }
    return MOD_RET_OK;       // false
}

/**
 * @brief Checks wether the buffer is full.
 * @param *buffer Pointer to buffer.
 * @return 1 if true, 0 if false.
 */
static int32_t bfr_is_full(bfr_gcode_t *buffer)
{
    if (buffer->length == buffer->size) {
        return 1;   // true
    }
    return MOD_RET_OK;       // false
}
