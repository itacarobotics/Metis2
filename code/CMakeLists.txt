cmake_minimum_required(VERSION 3.10)
project(Metis2)

# Set C standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Set the source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# Include directories
include_directories(
    ${SRC_DIR}/buffer
    ${SRC_DIR}/comunication
    ${SRC_DIR}/gcode
    ${SRC_DIR}/gripper_driver
    ${SRC_DIR}/inverse_geometry
    ${SRC_DIR}/module_template
    ${SRC_DIR}/motion_planner
    ${SRC_DIR}/motor_driver
    ${SRC_DIR}/robot
    ${SRC_DIR}/task_manager
    ${SRC_DIR}/trajectory_generator
)

# Source files
set(SOURCES
    ${SRC_DIR}/buffer/buffer.c
    ${SRC_DIR}/comunication/comunication.c
    ${SRC_DIR}/gcode/gcode.c
    ${SRC_DIR}/gripper_driver/gripper_driver.c
    ${SRC_DIR}/inverse_geometry/inverse_geometry.c
    ${SRC_DIR}/module_template/module_template.c
    ${SRC_DIR}/motion_planner/motion_planner.c
    ${SRC_DIR}/motor_driver/motor_driver.c
    ${SRC_DIR}/robot/robot.c
    ${SRC_DIR}/task_manager/task_manager.c
    ${SRC_DIR}/trajectory_generator/trajectory_generator.c
)

# Add the executable
add_executable(main 
    ${SOURCES}
    ${SRC_DIR}/main.c
)
target_link_libraries(main m)



# ## tests
# # Set the tests directory
# set(TESTS_DIR ${CMAKE_SOURCE_DIR}/tests)

# add_executable(ig_test 
#     ${SOURCES}
#     ${TESTS_DIR}/ig_test.c
# )
# target_link_libraries(ig_test m)
