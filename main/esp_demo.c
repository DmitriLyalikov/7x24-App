/**
 * @file    esp_demo.c
 * @author  Dmitri Lyalikov
 * @brief   7x24 Application Source for ESP32 
 * @version 0.1
 * @date    2022-03-07
 * @copyright Copyright (c) 2022
 * 
 *     ESP32 PIN TOPOLOGY
 *     -GPIO34 <-> LM35 Vout
 *     -GPIO32 <-> L298N IN1_0
 *     -GPIO33 <-> L298N IN2_0
 *     -GPIO25 <-> L298N EN1_0
 *     -GPIO27 <-> L298N IN1_1
 *     -GPIO14 <-> L298N IN2_1
 *     -GPIO12 <-> L298N EN1_1
 */

#include "esp_system.h"
#include "esp_spi_flash.h"
#include <stdio.h>
#include <sys/param.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "perfmon.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "L298N.h"

#define TEMP_SENSE_IO 4

#define DEFAULT_VREF 1500        // ADCout = (Vin, ADC*2^12)/Vref
#define NO_OF_SAMPLES 1

#define MIN_INTEGRAL 2.0         // Min integral value
#define MAX_INTEGRAL 10.0        // Max Integral Value
#define EPSILON      0.01

static const float kp = 2;       //Integral Ratio
static const float ki = 5;       // Differential ratio 1
static const float kd = 1;      // Sum of errors
static volatile float sum = 0;
static volatile float old_error = 0;

static float set_temp = 20;      // Desired Temperature of Target (Celsius)


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;   // GPIO34 
static const adc_bits_width_t width = ADC_WIDTH_12Bit;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static SemaphoreHandle_t xQueueMutex;

typedef struct xSense_t
{
    TickType_t xTimeStamp;
    float ulValue;
}xSense_t;

QueueHandle_t xSense_Queue;

/**
 * @brief Initialize Temperature Sense Mailbox Queue
 */
void vQueueInit(void)
{   
    xSense_Queue = xQueueCreate(1, sizeof(xSense_t));
}

void vUpdateQueue(float ulNewValue)
{
    xSense_t xData;
    xData.ulValue = ulNewValue;
    xData.xTimeStamp = pdTICKS_TO_MS(xTaskGetTickCount());
    xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(1000));
    xQueueOverwrite(xSense_Queue, &xData);
    xSemaphoreGive(xQueueMutex);
}

/**
 * @brief 
 *        Read Mailbox Queue
 * @param pxData Pointer to Struct of type xSense_t for contents to be copied
 */
static void vReadQueue(xSense_t *pxData)
{
    xQueuePeek(xSense_Queue, pxData, portMAX_DELAY);
}

/**
*@brief
*  Config ADC (GPIO 34)
*  Channel 6 Unit 1 
*  (12 Bit Resolution, 0 Attenuation, 1V2 Reference)
*/
static esp_err_t ADC_init()
{
    adc1_config_channel_atten((adc1_channel_t)channel, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    return ESP_OK;
}   

/**
*@brief
*  Read Temperature from LM35 Over ADC 
*  LM35 Transfer Function: Vout = 10mV/C *T
*  Where T is temperature in Celsius
*  Task Repeats every 1000ms
*  Take (NO_OF_SAMPLES) of reads and use average
*  Update xSense_Queue mailbox temp
*/
static void Temp_Sense()
{   
    for(;;){
    // TickType_t xTimeNow = xTaskGetTickCount();
    float temp = 0;
    temp = (adc1_get_raw((adc1_channel_t)channel));    
    temp = (esp_adc_cal_raw_to_voltage(temp, adc_chars));
    temp = temp * .1;
    xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(1000));
    // Update xSense_Queue
    vUpdateQueue(temp); 
    xSemaphoreGive(xQueueMutex);
    vTaskDelay(pdMS_TO_TICKS(12000));
    }
}

/**
*@brief
*  PID Computation Loop 
*  Task Repeats every 3000ms
*  Read xSense_Queue and calculate error
*  Sum P + I + D components 
*  PID output Range 0 - 100 (PWM Duty of L298N)
*  L298N Drives BYT-7A015 Pump
*  Minimum Voltage 8V
*/
static void vPIDCompute(void *pvParameter)
{   
    float PID_Output = 0;
    xSense_t pxData = {
            .ulValue = 0,
            .xTimeStamp = 0,
        };
    float up, ud = 0;
    float ui = 0;
    for(;;)
    {
        xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(2000));
        vReadQueue(&pxData);
        xSemaphoreGive(xQueueMutex);
        TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        TickType_t time_change = (now - pxData.xTimeStamp) / 1000;
        float error = -(set_temp - pxData.ulValue);
        printf("Read Temp: %f\n", pxData.ulValue);
        // Proportional Branch
        up = kp * error;
        // Integral Branch
        if (abs(error) > EPSILON){
            ui = ui + error * time_change;
        }
        ui = ki * ui;
        // Differential Branch
        ud = kd * (error - old_error) / time_change;
        old_error = error;
        // Return PID Response
        PID_Output = up + ui + ud;
        if (PID_Output > 100){
            PID_Output = 100;
            printf("Pump Max Speed Reached!\n");
        }
        if (PID_Output < 70){
            PID_Output = 0;
        }
        vL298N_control(PID_Output);
        printf("PID OUTPUT: %f\n", PID_Output);
        vTaskDelay(pdMS_TO_TICKS(12000));
    }
}

/**
*@brief
*  Entry Point to Application 
*  Configure ADC, L298N PWM, Queue
*  Assign Temp_Sense and xPIDCompute to Core 0
*/ 
void app_main(void)
{   
    printf("********ESP32 7x24 Application********\n");
    printf("***Temperature Set to %f Celsius***\n", set_temp);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(ADC_init());
    printf("***LM35 Module Initialized***\n");
    ESP_ERROR_CHECK(L298N_init());
    printf("***L298N PWM Module Initialized***\n");
    xQueueMutex = xSemaphoreCreateMutex();
    vQueueInit();
    xTaskCreatePinnedToCore(Temp_Sense,
                            "TEMP_SENSE",
                            600,
                            NULL,
                            0, 
                            NULL,
                            0);
    xTaskCreatePinnedToCore(&vPIDCompute,
                            "PID_Compute",
                            1750,
                            NULL,
                            0,
                            NULL,
                            0); 
}
