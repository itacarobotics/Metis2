#include "buffer.h"

template <typename data_t>
Buffer<data_t>::Buffer(int32_t buffer_size)
{
    this->buffer_size = buffer_size + 1;        // "wasting" a buffer slot
    data_t *data = new data_t[buffer_size];

    head_idx = 0;
    tail_idx = 0;
    return;
}


template <typename data_t>
void Buffer<data_t>::add_item(data_t item)
{
    data[head_idx] = item;
    head_idx = (head_idx + 1) % buffer_size;
    return;
}


template <typename data_t>
void Buffer<data_t>::get_item(data_t *item)
{
    *item = data[tail_idx];
    tail_idx = (tail_idx + 1) % buffer_size;
    return;
}


template <typename data_t>
buffer_state_t Buffer<data_t>::get_state(void)
{
    if ( (head_idx + 1) % buffer_size == tail_idx ) {
        return BUFFER_FULL;
    }
    else if ( head_idx == tail_idx ) {
        return BUFFER_EMPTY;
    }

    return BUFFER_READY;
}


template <typename data_t>
void Buffer<data_t>::reset(void)
{
    head_idx = 0;
    tail_idx = 0;
    return;
}