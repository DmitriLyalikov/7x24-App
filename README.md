
# 7x24 ESP32 Application

This application controls the temperature of target submerged in dielectric fluid with a PID algorithm given feedback from an LM35 temperature sensor.
The PID output is a PWM signal to water pump to increase/decrease flow rate consequentially controlling heat transfer process. Temperature, Flow rate and other process data are shared via LCD display and AWS IoT dashboard via MQTT over WiFi conection. 

## Author
Dmitri Lyalikov

## Hardware Components
ESP32-DevKitC-V4.

L298N DC-DC Motor Driver.

LM35 Analog Temperature Sensor.

GR 1/4 Flow Sensor

LCD1602 

## ESP32 Pin Topology
*     -GPIO34 <-> LM35 Vout
*     -GPIO32 <-> L298N IN1_0
*     -GPIO33 <-> L298N IN2_0
*     -GPIO25 <-> L298N EN1_0
*     -GPIO21 <-> GR 1/4 Flow Sensor VO
*     -GPIO19 <-> LCD1602 SCL
*     -GPIO18 <-> LCD1602 SDA

## Projects contents

The project **7x24-App** contains one source file in C language [esp_demo.c](main/esp_demo.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── components
    ├──L298N
    ├──esp32-i2c-lcd1602
    ├──awsiot
    ├──esp32-smbus
      
├── main
│   ├── CMakeLists.txt
│   ├── component.mk           Component make file
│   └── esp-demo.c
├── Makefile                   Makefile used by legacy GNU Make
└── README.md                  This is the file you are currently reading
```
