#ifndef LM35_H
#define LM35_H

#include "driver/adc.h"
#include "freertos/semphr.h"
#include "sys_resource.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"

#define NO_OF_SAMPLES CONFIG_NO_OF_SAMPLES
#define SAMPLE_PERIOD CONFIG_SAMPLE_PERIOD
#define DEFAULT_VREF 1500        // ADCout = (Vin, ADC*2^12)/Vref

static esp_err_t ADC_init();

static void Temp_Sense(QueueHandle_t xSenseQueue, SemaphoreHandle_t xQueueMutex);

#endif