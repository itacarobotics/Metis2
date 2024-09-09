#include <iostream>
#include <chrono>

#include "../include/inverse_geometry.h"

int main()
{
    InverseGeometry ig;
    position_t pos_des;
    joints_t q;
    bool rc;

    pos_des.x = -40;
    pos_des.y = 30;
    pos_des.z = -100;


    // Get the starting point of time
    auto start = std::chrono::high_resolution_clock::now();

    // Call the function whose execution time we want to measure
    rc = ig.inverse_geometry(&pos_des, &q);

    // Get the ending point of time
    auto end = std::chrono::high_resolution_clock::now();

    if (!rc) {
        std::cout << "[ERROR!] No solution found.\n";
        return -1;
    }

    // Calculate the duration (difference between start and end)
    std::chrono::duration<double> elapsed = end - start;

    // Print the computation time in seconds
    std::cout << "Computation time: " << elapsed.count() << " seconds" << std::endl;

    std::cout << "q1: " << q.q1 << "\n";
    std::cout << "q2: " << q.q2 << "\n";
    std::cout << "q3: " << q.q3 << "\n";
}