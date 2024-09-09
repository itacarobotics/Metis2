#ifndef CONFIGURATION_Hù
#define CONFIGURATION_H


// robot's kinematic properties
#define BASE_RADIUS                     40      // [ mm ]
#define BICEPS_LENGTH                   80      // [ mm ]
#define FOREARM_LENGTH                  160     // [ mm ]
#define EE_RADIUS                       25      // [ mm ]

// robot's dynamics properties
#define MAX_LINEAR_VEL                  500     // [ mm/s ]
#define MAX_LINEAR_ACC                  2000    // [ mm/s2 ]
#define MAX_ROTATION_VEL                20      // [ rad/s ]
#define MAX_ROTATION_ACC                50      // [ rad/s2 ]

// path resolution
#define PATH_STEP_DISTANCE              2       // [ mm ]
#define ROTATION_STEP_DISTANCE          0.01    // [ rad ]


#endif  // CONFIGURATION_H