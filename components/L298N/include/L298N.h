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
#ifndef L298N_H_
#define L298N_H_

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/mcpwm.h"

void vL298N_control(float duty);

esp_err_t L298N_init(void);

void vSuspend_PWM(void);

#endif