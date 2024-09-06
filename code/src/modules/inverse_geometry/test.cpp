#include <iostream>
#include "inverse_geometry.cpp"


int main()
{
    position_t pos_des;
    joints_t q;

    pos_des.x = -40;
    pos_des.y = 30;
    pos_des.z = -180;


    bool rc;
    rc = inverse_geometry(&pos_des, &q);
    if (!rc) {
        std::cout << "[ERROR!] No solution found.\n";
        return -1;
    }

    std::cout << "q1: " << q.q1 << "\n";
    std::cout << "q2: " << q.q2 << "\n";
    std::cout << "q3: " << q.q3 << "\n";
}