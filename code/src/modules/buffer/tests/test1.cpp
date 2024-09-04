#include <iostream>
#include "stdint.h"

#include "../buffer.h"


#define BUFFER_SIZE   4

int main()
{
    Buffer<int32_t> buffer1(BUFFER_SIZE);
    int32_t item;
    int32_t rc;

    for (int32_t i = 0; i < 120; ++i ) {
        if (buffer1.get_state() == BUFFER_FULL) {
            std::cout << "[ERROR!] Buffer is full." << std::endl;
            break;
        }

        item = i*12;                // do computation
        buffer1.add_item(item);
        std::cout << "added item: " << item << std::endl;
    }

    for (int32_t i = 0; i < 9; ++i ) {
        if (buffer1.get_state() == BUFFER_EMPTY) {
            std::cout << "[ERROR!] Buffer is empty." << std::endl;
            break;
        }

        buffer1.get_item(&item);
        std::cout << "buffer [" << i << "]: " << item << std::endl;     // use item
    }

    return 0;
}