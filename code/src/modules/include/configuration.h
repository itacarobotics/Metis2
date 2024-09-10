#ifndef CONFIGURATION_Hù
#define CONFIGURATION_H


// robot's kinematics properties
#define BASE_RADIUS                     40      // [ mm ]
#define BICEPS_LENGTH                   80      // [ mm ]
#define FOREARM_LENGTH                  160     // [ mm ]
#define EE_RADIUS                       25      // [ mm ]

#define JOINT_LIMIT_MIN                -999
#define JOINT_LIMIT_MAX                 999

#define POSITION_HOME_OFFSET_Z         -80

// robot's dynamics properties
#define MAX_LINEAR_VEL                  500     // [ mm/s ]
#define MAX_LINEAR_ACC                  2000    // [ mm/s2 ]
#define MAX_ROTATION_VEL                20      // [ rad/s ]
#define MAX_ROTATION_ACC                50      // [ rad/s2 ]

// period between via points
#define VIA_POINTS_TIME_STEP            0.01    // [ s ]


#endif  // CONFIGURATION_H