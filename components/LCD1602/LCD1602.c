/**
 * @file L298N.c
 * @author Dmitri Lyalikov
 * @brief LCD1602 configurations and task
 * @version 0.1
 * @date 2022-03-02
 * @copyright Copyright (c) 2022
 * 
 */

#include "LCD1602.h"

static const char* TAG = "Display {LCD-1602}";

static QueueHandle_t xTemp_Queue;
static QueueHandle_t xFlow_Queue;
static SemaphoreHandle_t xQueueMutex;

void vInit_Display_Task(QueueHandle_t xTemp_Queue, QueueHandle_t xFlow_Queue, SemaphoreHandle_t xQueueMutex){
    ESP_LOGI( TAG, "Starting LCD Display Task");
    xTaskCreatePinnedToCore(&vDisplay_Task,
                        "LCD_Display",
                        3000,
                        NULL,
                        1,
                        NULL,
                        1);  
}

/**
 * @brief LCD1602 Task 
 * Display Temperature and Flow Rate
 * Update every PERIOD seconds
 */
void vDisplay_Task(void *pvParameters)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    // Init SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    // Init LCD1602 device 
    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));
    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));
    i2c_lcd1602_set_backlight(lcd_info, true);

    i2c_lcd1602_write_string(lcd_info, "Temp (C):");
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_write_string(lcd_info, "Flow(L/M):");
    char read_string[50];
    xSense_t pxData = {
        .ulValue = 0,
        .xTimeStamp = 0,
    };
    for (;;)
    {
        xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(2000));
        vReadQueue(&pxData, xTemp_Queue);
        xSemaphoreGive(xQueueMutex);
        sprintf(read_string, "%d", pxData.ulValue);
        i2c_lcd1602_move_cursor(lcd_info, 9, 0);
        i2c_lcd1602_write_string(lcd_info, read_string);
        i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DEGREE);

        xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(2000));
        vReadQueue(&pxData, xFlow_Queue);
        xSemaphoreGive(xQueueMutex);
        sprintf(read_string, "%d", pxData.ulValue);
        i2c_lcd1602_move_cursor(lcd_info, 10, 1);
        i2c_lcd1602_write_string(lcd_info, read_string);

        vTaskDelay(pdMS_TO_TICKS(LCD1602_REFRESH_PERIOD)); 
    }
}