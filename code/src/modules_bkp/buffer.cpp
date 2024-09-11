#include "include/buffer.h"

template <typename data_t>
Buffer<data_t>::Buffer(int32_t buffer_size)
{
    this->buffer_size = buffer_size;
    data_t *data = new data_t[buffer_size];

    head_idx        = 0;
    tail_idx        = 0;
    buffer_length   = 0;
    mutex           = true;
    return;
}


template <typename data_t>
bool Buffer<data_t>::produce(data_t item)
{
    if ( buffer_length == buffer_size ) {
        return false;   // the buffer is full
    }
    if (!mutex) {
        return false;   // the consumer is taking an item
    }

    mutex = false;      // lock process

    data[head_idx] = item;
    head_idx = (head_idx + 1) % buffer_size;
    buffer_length += 1;

    mutex = true;       // unlock process

    return true;        // succesfully added new item to buffer
}


template <typename data_t>
bool Buffer<data_t>::consume(data_t *item)
{
    if ( buffer_length == 0 ) {
        return false;   // the buffer is empty
    }
    if (!mutex) {
        return false;   // the producer is adding an item
    }

    mutex = false;      // lock process

    *item = data[tail_idx];
    tail_idx = (tail_idx + 1) % buffer_size;
    buffer_length -= 1;

    mutex = true;       // unlock process

    return true;        // succesfully taken item from buffer
}


template <typename data_t>
bool Buffer<data_t>::is_full(void)
{
    if ( buffer_length == buffer_size ) {
        return true;
    }
    return false;
}

template <typename data_t>
bool Buffer<data_t>::is_empty(void)
{
    if ( buffer_length == 0 ) {
        return true;
    }
    return false;
}


template <typename data_t>
void Buffer<data_t>::reset(void)
{
    buffer_length   = 0;
    head_idx        = 0;
    tail_idx        = 0;
    return;
}