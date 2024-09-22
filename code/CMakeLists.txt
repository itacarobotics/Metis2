cmake_minimum_required(VERSION 3.10)
project(Metis2)

# Set C standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Set the modules directory
set(MOD_DIR ${CMAKE_SOURCE_DIR}/src/modules)

# Set the app directory
set(APP_DIR ${CMAKE_SOURCE_DIR}/src/app)


# Include directories
include_directories(
    ${MOD_DIR}/include
)
include_directories(
    ${APP_DIR}/include
)

# Source files
set(SOURCES
    ${MOD_DIR}/buffer/buffer.c
    # ${MOD_DIR}/comunication/comunication.c
    ${MOD_DIR}/gcode/gcode.c
    # ${MOD_DIR}/gripper_driver/gripper_driver.c
    ${MOD_DIR}/inverse_geometry/inverse_geometry.c
    # ${MOD_DIR}/motor_driver/motor_driver.c
    ${MOD_DIR}/robot/robot.c
    ${MOD_DIR}/trajectory_generator/trajectory_generator.c
    
    ${APP_DIR}/fsm_middleware.c
)

# Add the executable
add_executable(main 
    ${SOURCES}
    ${APP_DIR}/app_main.c
)
target_link_libraries(main m)



# # ## tests
# # # Set the tests directory
# set(TESTS_DIR ${CMAKE_SOURCE_DIR}/tests)

# # Add the executable
# add_executable(buffer_test 
#     ${SOURCES}
#     ${TESTS_DIR}/buffer_test.c
# )
# target_link_libraries(buffer_test m)
