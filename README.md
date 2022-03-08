
# 7x24 ESP32 Application Code

This application controls heating element temperature with a PID control

# Author
Dmitri Lyalikov

# Hardware Components
ESP32-DevKitC-V4
L298N DC-DC Motor Driver
LM35 Analog Temperature Sensor

# ESP32 Pin Topology
*     -GPIO34 <-> LM35 Vout
*     -GPIO32 <-> L298N IN1_0
*     -GPIO33 <-> L298N IN2_0
*     -GPIO25 <-> L298N EN1_0
*     -GPIO27 <-> L298N IN1_1
*     -GPIO14 <-> L298N IN2_1
*     -GPIO12 <-> L298N EN1_1

# Components
esp_demo.c 
