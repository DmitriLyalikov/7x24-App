#ifndef GR2048_H
#define GR2048_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sys_resource.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#define FLOW_RATE_PIN  CONFIG_FLOW_RATE_PIN
/**
 * @brief Flow Sensor interrupt handler
 *  increment counter each fire
 * @param arg 
 */
void IRAM_ATTR vFlow_ISR_Handler(void* arg);

/**
 * @brief Flow Rate Task to Read From GR-4028
 *        Writes to xFlow_Queue every 60 seconds
 * Flow_Rate = (Pulse Frequency) / 38 (Flow Rate in liters per minute)
 */
void vFlowRate_Task(void *pvParameters);

/**
 * @brief Initialize FLOW_GPIO as Input for Flow Rate Sensor
 *        Config vFlowInterrupt_Handler as ISR to increment 
 *        flow_samples
 */
void vInit_FlowRate_Task(QueueHandle_t xFlow_Queue, SemaphoreHandle_t xQueueMutex);

#endif