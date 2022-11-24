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
#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "L298N.h"
#include "sys_resource.h"

#define SET_TEMP           CONFIG_SET_TEMP
#define PID_CONTROL_PERIOD CONFIG_PID_CONTROL_PERIOD
#define PID_KP             CONFIG_PID_KP
#define PID_KI             CONFIG_PID_KI
#define PID_KD             CONFIG_PID_KD
#define PID_EPSILON        CONFIG_PID_EPSILON
#define PID_MAX_INTEGRAL   CONFIG_PID_MAX_INTEGRAL
#define PID_MIN_INTEGRAL   CONFIG_PID_MIN_INTEGRAL

void vInit_PIDCompute_Task(QueueHandle_t xSenseQueue, SemaphoreHandle_t xQueueMutex); 

void vPIDCompute_Task(void *pvParameter);

#endif