#include <iostream>
#include "stdint.h"

#include "../include/buffer.h"


#define BUFFER_SIZE   5

int main()
{
    Buffer<int32_t> buffer1(BUFFER_SIZE);
    int32_t item;
    bool rc;

    for (int32_t i = 0; i < 100; i++) {

        if (buffer1.is_full()) {
            std::cout << "[ERROR!] buffer is full!\n";
            break;
        }
        
        item = i * 3;       // compute calculations

        rc = buffer1.produce(item);

        if (rc == false) {
            std::cout << "[ERROR!] item has not been added!\n";
            break;
        }

        std::cout << "item[" << i  << "]: " << item << "\n";
    }

    for (int32_t i = 0; i < 100; i++) {
        rc = buffer1.consume(&item);

        if (rc == false) {
            std::cout << "[ERROR!] buffer is empty!\n";
            break;
        }

        std::cout << "item[" << i  << "]: " << item << "\n";
    }

    return 0;
}