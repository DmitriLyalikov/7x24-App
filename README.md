
# 7x24 ESP32 Application

## System Overview

This is the software application that is part of the 2022 design for Manhattan College in the 7x24 Senior Design Competition for processor cooling (heat dissipation) systems in the context of data centers. Traditional solutions to the problem of cooling involve heatsinks and fans. With the goal of increasing energy-efficiency and scalabilty, our design involved a two-phase immersive cooling system using a dielectric fluid with a much lower boiling point than water to significantly reduce heat dissipation costs. 

To do this, the desired and current temperature of the processor must be logged, and a greater deviation between the two requires more flow of cold water through the system to exchange heat from the dielectric to an exchange tank. Of course, this system can simply be ran at full speed, which will give great results, however, the heat exchange and pumping system is an energy intensive task. 

Using a PID algorithm, will use three feedback components:
* Proportional Component, difference between set and current temperatures

$$ P = current - set  $$

* Derivative Component, numerically approximated current rate of change of temperature

$$ D = dT/dt $$

* Integral Component, the sum of previous temperature deviations

$$ D = \int\limits_0^t T dt $$

These components are then summed together and give the response of the system, in the form of a percentage from 0 - 100 % flow rate. This response output is a PWM signal to the EN pin of the DC motor controller (L298N). This has the effect of controlling the speed of the pump flow rate.

Each of these components are tuned on physical system with tuning parameters. These change the responsiveness of each of the components, so that the set temperature of the processor is reached using a minimal amount of pump time. For detailed introduction to tuning the PID parameters, use this resource: https://pidexplained.com/how-to-tune-a-pid-controller/
 
The project presentation can be found at: https://www.youtube.com/watch?v=ilWbGFTbL-w

## Hardware Components
ESP32-DevKitC-V4.

L298N DC-DC Motor Driver.

LM35 Analog Temperature Sensor.

GR 1/4 Flow Sensor

LCD1602 Liquid Crystal Display

<img width="903" alt="image" src="https://user-images.githubusercontent.com/68623356/198076812-36c3dcf1-d0e3-4465-bee2-6a430304924f.png">


## Using this Application
This application is created in the esp-idf framework. The toolchain, compiler, drivers, all can be installed locally here: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

This application was developed on a Windows system, and the windows installer: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html can be found here that automates the setup process. 
Upon installation and following these steps, a ESP_IDF 4.x CMD utility will be installed, which is a development environment for esp-idf based projects. Open this. I used version 4.4, but a newer version may be available upon completion of this project.
### Configure ESP-IDF
```console
install
export.bat
```
### Configure and build this project
```console
cd projects
git clone https://github.com/DmitriLyalikov/7x24-App.git
cd 7x24-App
idf.py fullclean
idf.py set-target ESP32
idf.py build
```

### Flashing and Running
With the ESP32 connected via USB to your host machine, connected to 'COMx'
```console
idf.py flash monitor 'COMx'
```
This will flash the application onto the system, and restart, it will then provide serial access to the application. 

### Testing 

In order to extract long term data for visualization, there are some python scripts for data logging in the /pylog subdirectory.

In order to use these scripts. Follow the procedure of flashing and running on and verify that logging is visible to the host machine. While the device is still connected, open a separate terminal utility in the pylog subdirectory, and use the command:

```console
python3 logger.py {COMx} {Outputfile_name} {testDuration}
```
These arguments are desribed as follows:

* COMx: The COM device using between host and ESP32 device
* Outputfile_Name: The name of the resulting data file generated, it will be saved in /pylog
* testDuration: Time in minutes to proceed with system monitoring and logging

An example usage would be (starting in project directory as before):
```console
cd pylog
python3 logger.py COM10 Dmitri_SimpleTest_Run 1
```
This will open a monitoring instance on COM device 10 for 1 minute and output data to a .csv file named Dmitri_SimpleTest_Run.csv. 

This will extract temperature and flow rate data at discrete time points from the system during runtime and generate a csv spreadsheet time markers. It will also build some useful plots for data visualization. The default .csv file will be saved in the same /pylog subdirectory.


### Configure Application settings
Some configurations may need to be made when using this application. Wifi SSID, Password, and authentication mode may/need be set before using this app, or the defaults will be used. Generally each component will have a configuration manager that is defined in components/component/Kconfig.Projbuild . Instead of editing hard-coded values in each component source file, a menuconfig utility is provided that can edit these values via command utility:

```console
idf.py menuconfig
```

<img width="640" alt="image" src="https://user-images.githubusercontent.com/68623356/203680072-d12c500b-6641-4c9f-9866-dffc7e808b5a.png">

After exiting and saving this graphical config utility. Make sure to build and reflash the utility again for changes to take effect: 
```console
idf.py reconfigure
idf.py build 
idf.py flash monitor 'COMx'
```
#### Tuning PID Constants

Part of an important test for this system is to configure properly the PID constants with different temperature set points mentioned before. This must be done iteratively while running the system. In the menuconfig utility discussed above, these parameters can be viewed without inspecting the source code and changed quickly. Using this with the logging python automation. A very quick tuning automation system can be made that can tweak each component, gather data, visualize the PID output, and repeat, until the optimal values are found. 

<img width="640" alt="image" src="https://user-images.githubusercontent.com/68623356/203680707-4e6135f3-5640-4b4d-9cd8-26bd1d7d6674.png">


### ESP32 Default Pin Connections
The default pin connections can be found and reconfigured or using. This should not need to be done unless porting this application to another system

*     -GPIO34 <-> LM35 Vout 
*     -GPIO32 <-> L298N IN1_0
*     -GPIO33 <-> L298N IN2_0
*     -GPIO25 <-> L298N EN1_0
*     -GPIO21 <-> GR 1/4 Flow Sensor VO
*     -GPIO19 <-> LCD1602 SCL
*     -GPIO18 <-> LCD1602 SDA

## Projects contents
If you need to look through the source code to understand more deeply, this is the an explanation of how this application is setup.
Also, if you notice an issue, misbehavior, mistake, or have a comment, thought, or advice, please open an issue in this github project with a description to the best you can. 
The project **7x24-App** contains one source file in C language [esp_demo.c](main/esp_demo.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both). 

Below is short explanation of remaining files in the project folder.
All components that are hardware facing (drivers for a specfic device) are named after the device they interact with (ie: LM35)

```
├── CMakeLists.txt
├── components
    ├──L298N                Init and control functions for the L298N motor controller using PWM
    ├──LM35                 Init and Task functions for the LM35 Temperature Sensor using ADC
    ├──LCD1602              Application Level Driver and Task for LCD1602 display
    ├──GR2048               Init and Task functions for the GR-2048 Flow Rate Sensor
    ├──sys_resource         System resources for task management, queues, structs
    ├──pid_controller       PID setup and computation task for control system with weights 
    ├──esp32-i2c-lcd1602    
    ├──esp32-smbus
      
├── main
│   ├── CMakeLists.txt
│   ├── component.mk    
│   └── esp-demo.c         Entry point to application
├── Makefile                   Makefile used by legacy GNU Make
└── README.md                  This is the file you are currently reading
```
## Finding and Reporting Issues
If you notice an issue, misbehavior, mistake, or have a comment, thought, or advice, please open an issue in this github project in the Issues Tab with a description to the best you can. This is the best way to have this resolved. 
Please add as much information as you can with at least one of the following:
* A description of the issue and the state of the system when it happened
* steps to recreate the same issue
* Desired or expected outcome
