#include <iostream>
#include <fstream>
#include <chrono>


#include "../include/trajectory_generator.h"


int main()
{
    // Open file to save data for gnuplot
    std::ofstream data_file("trj_data.csv");

    if (!data_file.is_open()) {
        std::cerr << "Failed to open file.\n";
        return -1;
    }


    TrajectoryGenerator tg;
    position_t pos_start;
    position_t pos_end;
    position_t pos;
    bool rc;
    

    /**********************************
     *          TRAJECTORY 1
     **********************************/
    std::cout<<"\n****** TRAJECTORY 1 ******\n\n";

    pos_start.x = 0;
    pos_start.y = 0;
    pos_start.z = 0;
    pos_start.k = 0;
    pos_start.t = 0;

    pos_end.x = 100;
    pos_end.y = 100;
    pos_end.z = 100;
    pos_end.k = 2.0;
    pos_end.t = -1;

    // Set trajectory
    rc = tg.set_trajectory_ptp(pos_start, pos_end);

    if (!rc) {
        std::cout << "[ERROR!] Invalid trajectory.\n";
        return -1;
    }

    while (true) {
        rc = tg.get_next_via_point(&pos);

        if (rc == false) {
            break;
        }

        // Save via points to file
        data_file << pos.x << " " << pos.y << " " << pos.z << " " << pos.k << " " << pos.t << "\n";
        std::cout << pos.x << " " << pos.y << " " << pos.z << " " << pos.k << " " << pos.t << "\n";
    }


    /**********************************
     *          TRAJECTORY 2
     **********************************/
    std::cout<<"\n****** TRAJECTORY 2 ******\n\n";

    pos_start = pos_end;
    pos_start.t = 0;

    pos_end.x = -100;
    pos_end.y = 100;
    pos_end.z = 100;
    pos_end.k = -2.0;
    pos_end.t = -1;

    // Set trajectory
    rc = tg.set_trajectory_ptp(pos_start, pos_end);

    if (!rc) {
        std::cout << "[ERROR!] Invalid trajectory.\n";
        return -1;
    }

    while (true) {
        rc = tg.get_next_via_point(&pos);

        if (rc == false) {
            break;
        }

        // Save via points to file
        data_file << pos.x << " " << pos.y << " " << pos.z << " " << pos.k << " " << pos.t << "\n";
        std::cout << pos.x << " " << pos.y << " " << pos.z << " " << pos.k << " " << pos.t << "\n";
    }


    data_file.close();
    return 0;
}
