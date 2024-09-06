#ifndef BUFFER_H
#define BUFFER_H

#include "stdint.h"


template <typename data_t>
class Buffer
{
public:
    Buffer(int32_t buffer_size);
    bool            produce(data_t item);
    bool            consume(data_t *item);
    bool            is_full(void);
    bool            is_empty(void);
    void            reset(void);

private:
    data_t          *data;
    int32_t         head_idx;
    int32_t         tail_idx;
    int32_t         buffer_size;
    int32_t         buffer_length;
    bool            mutex;          // semaphore to ensure syncronisation
};


#include "buffer.cpp"

#endif // BUFFER_H
