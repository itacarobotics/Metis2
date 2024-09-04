#ifndef BUFFER_H
#define BUFFER_H

#include "stdint.h"


enum buffer_state_t {
    BUFFER_EMPTY,
    BUFFER_FULL,
    BUFFER_READY
};


template <typename data_t>
class Buffer
{
public:
    Buffer(int32_t buffer_size);
    void            add_item(data_t item);
    void            get_item(data_t *item);
    buffer_state_t  get_state(void);
    void            reset(void);

private:
    data_t          *data;
    int32_t         head_idx;
    int32_t         tail_idx;
    int32_t         buffer_size;
};


#include "buffer.cpp"

#endif // BUFFER_H
