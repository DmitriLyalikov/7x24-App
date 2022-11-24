#ifndef LM35_H
#define LM35_H

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "freertos/semphr.h"
#include "sys_resource.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"

#define NO_OF_SAMPLES CONFIG_NO_OF_SAMPLES
#define SAMPLE_PERIOD CONFIG_SAMPLE_PERIOD
#define DEFAULT_VREF 1500        // ADCout = (Vin, ADC*2^12)/Vref

void vInit_TempSense_Task(QueueHandle_t xSenseQueue, SemaphoreHandle_t xQueueMutex);

void vTempSense_Task(void *pvParameters);

#endif