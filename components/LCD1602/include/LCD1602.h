/**
 * @file L298N.h
 * @author Dmitri Lyalikov
 * @brief L298N Driver Library
 * @version 0.1
 * @date 2022-03-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef LCD1602_H_
#define LCD1602_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "i2c-lcd1602.h"
#include "smbus.h"
#include "sys_resource.h"

// LCD1602
#define LCD_NUM_ROWS               2
#define LCD_NUM_COLUMNS            32
#define LCD_NUM_VISIBLE_COLUMNS    16

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL
#define LCD1602_REFRESH_PERIOD   CONFIG_LCD1602_REFRESH_PERIOD

void vDisplay_Task_Init(QueueHandle_t xTemp_Queue, QueueHandle_t xFlow_Queue, SemaphoreHandle_t xQueueMutex);
void vDisplay_Task(void *pvParameters);

#endif