
# 7x24 ESP32 Application

This application controls the temperature of target submerged in dielectric fluid with a PID algorithm given feedback from an LM35 temperature sensor.
The PID output is a PWM signal to water pump to increase/decrease flow rate consequentially controlling heat transfer process. Temperature, Flow rate and other process data are shared via LCD display and AWS IoT dashboard via MQTT over WiFi conection. 

The project presentation can be found at: https://www.youtube.com/watch?v=ilWbGFTbL-w

## Author
Dmitri Lyalikov

## Hardware Components
ESP32-DevKitC-V4.

L298N DC-DC Motor Driver.

LM35 Analog Temperature Sensor.

GR 1/4 Flow Sensor

LCD1602 Liquid Crystal Display

<img width="903" alt="image" src="https://user-images.githubusercontent.com/68623356/198076812-36c3dcf1-d0e3-4465-bee2-6a430304924f.png">


## Using this Project
* Download and Install the ESP-IDF toolchain: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
* After installing, open the ESP-IDF terminal application and navigate to /projects directory
* Clone this repository using 
```console
git clone https://github.com/DmitriLyalikov/7x24-App.git
cd 7x24-App
idf.py configure
idf.py build
```

* To flash and access serial output at runtime: 
```console
idf.py flash monitor {COMxxx}
```

## ESP32 Default Pin Connections
The default pin connections can be found and reconfigured in the /main/KConfig.ProjBuild or using 
```console
idf.py menuconfig
```

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
All components that are hardware facing (drivers for a specfic device) are named after the device they interact with (ie: LM35)

```
├── CMakeLists.txt
├── components
    ├──L298N                Init and control functions for the L298N motor controller using PWM
    ├──LM35                 Init and Task functions for the LM35 Temperature Sensor using ADC
    ├──GR2048               Init and Task functions for the GR-2048 Flow Rate Sensor
    ├──sys_resource         System resources for task management, queues, structs
    ├──esp32-i2c-lcd1602    
    ├──esp32-smbus
      
├── main
│   ├── CMakeLists.txt
│   ├── component.mk    
│   |──Kconfigu.Projbuild
│   └── esp-demo.c
├── Makefile                   Makefile used by legacy GNU Make
└── README.md                  This is the file you are currently reading
```
