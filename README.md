# Metis2
Metis2 is the latest enhanced version of Metis. This project is currently under development, however below is an overview of the changes.

## Mechanics
The new delta robot provides a modular design, with less components and assembly time. An extra degree of freedom has been added to the end effector to rotate around the z-axis. Furthermore, the size of the new robot is significantly smaller than the previous one, therefore it can better fits in smaller environments, such as a small production line.


## Electronics
The control unit of the robot is the BIGTREETECH-SKR-mini-E3, which is a main board for 3D printers. Beyond that, a custom PCB has been designed as a shield for the main board, in order to provide GPIOs and power supply for encoders and motors using terminal blocks.


## Firmware
The control software of the robot is subdivided in two parts based on abstraction levels. The low-level firmware runs on an STM32-G01B1RCT6 micro-controller integrated onto the motherboard. The firmware runs different state machines, each responisble for the comunication, processing data for the trajectory generation and inverse geometry and controlling the closed loop stepper motors. The higher level software controls the position of the robot using the GCODE format.