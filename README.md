
# 7x24 ESP32 Application Code

This application controls heating element temperature with a PID control

## Author
Dmitri Lyalikov

## Hardware Components
ESP32-DevKitC-V4.

L298N DC-DC Motor Driver.

LM35 Analog Temperature Sensor.

## ESP32 Pin Topology
*     -GPIO34 <-> LM35 Vout
*     -GPIO32 <-> L298N IN1_0
*     -GPIO33 <-> L298N IN2_0
*     -GPIO25 <-> L298N EN1_0
*     -GPIO27 <-> L298N IN1_1
*     -GPIO14 <-> L298N IN2_1
*     -GPIO12 <-> L298N EN1_1

## Projects contents

The project **7x24-App** contains one source file in C language [esp_demo.c](main/esp_demo.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── components
    ├──L298N
       ├── include
           ├── L298N.h
       ├── L298N.c
       ├── component.mk 
       ├── CMakeLists.txt   
├── main
│   ├── CMakeLists.txt
│   ├── component.mk           Component make file
│   └── esp-demo.c
├── Makefile                   Makefile used by legacy GNU Make
└── README.md                  This is the file you are currently reading
```
