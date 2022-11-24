/**
 * @file    7x24_App.c
 * @author  Dmitri Lyalikov
 * @brief   7x24 Application Source for ESP32 
 * @version 2.1
 * @date    2022-11-23
 * @copyright Copyright (c) 2022
 * 
 * Entry Point and Application, App_Main():
 * 
 *     ESP32 PIN TOPOLOGY
 *     -GPIO34 <-> LM35 Vout
 *     -GPIO32 <-> L298N IN1_0
 *     -GPIO33 <-> L298N IN2_0
 *     -GPIO25 <-> L298N EN1_0
 *     -GPIO27 <-> L298N IN1_1
 *     -GPIO14 <-> L298N IN2_1
 *     -GPIO12 <-> L298N EN1_1
 *     -GPIO21 <-> G 1/4 Flow Sensor
 */
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "nvs_flash.h"

#include "sys_resource.h"
#include "GR2048.h"
#include "LCD1602.h"
#include "L298N.h"
#include "LM35.h"
#include "pid_controller.h"

static const char* TAG = "7x24 APP";

// Global resource queue handles 
static SemaphoreHandle_t xQueueMutex;
QueueHandle_t xSense_Queue, xFlow_Queue;

/**
 * @brief Initialize I2C Master bus for LCD1602 Display
 */
static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

/**
*@brief
*  Entry Point to Application 
*  Initialize Hardware components and application tasks
*/ 
void app_main(void)
{   
    nvs_flash_init();
    ESP_ERROR_CHECK(nvs_flash_init());

    xQueueMutex = xSemaphoreCreateMutex();
    xSense_Queue = vQueueInit();
    xFlow_Queue = vQueueInit();

    i2c_master_init();
    ESP_ERROR_CHECK(L298N_init());

    ESP_LOGI(TAG, "Spawning Tasks");
    vInit_TempSense_Task(xSense_Queue, xQueueMutex);
    vInit_FlowRate_Task(xFlow_Queue, xQueueMutex);
    vInit_Display_Task(xSense_Queue, xFlow_Queue, xQueueMutex);
    vInit_PIDCompute_Task(xSense_Queue, xQueueMutex);
}
