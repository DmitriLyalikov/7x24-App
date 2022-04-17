/**
 * @file L298N.c
 * @author Dmitri Lyalikov
 * @brief L298N Driver Source for Controlling DC-DC Motor Controller With PWM 
 * @version 0.1
 * @date 2022-03-02
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "L298N.h"
#include "sdkconfig.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"


static const gpio_num_t SERVO0_IN1_GPIO = CONFIG_SERVO0_IN1_GPIO;
static const gpio_num_t SERVO0_IN2_GPIO = CONFIG_SERVO0_IN2_GPIO;
static const gpio_num_t SERVO0_ENABLE_GPIO = CONFIG_SERVO0_ENABLE_GPIO;

/**
 * @brief 
 *     Initialize and commit MCPWM configurations
 *     MCPWM Unit 0, Timer 0, 
 *     Frequency 30kHz
 */
esp_err_t L298N_init(void)
{
    gpio_set_direction(SERVO0_IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SERVO0_IN1_GPIO, 1);
    printf("Set Pin %d to High\n", SERVO0_IN1_GPIO);
    gpio_set_direction(SERVO0_IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SERVO0_IN2_GPIO, 0);
    printf("Set Pin %d to Low\n", SERVO0_IN2_GPIO);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO0_ENABLE_GPIO);
    printf("Set Pin %d to PWM OUTPUT\n", SERVO0_ENABLE_GPIO);


    mcpwm_config_t pwm_config = {
        .frequency     = 30000,
        .cmpr_a        = 0,
        .counter_mode  = MCPWM_UP_COUNTER,
        .duty_mode     = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    
    return ESP_OK;
}

/**
 * @brief 
 * Set PWM Unit 0 Duty Cycle 
 * 100 is equivalent to mcpwm_set_signal_high()
 * 0 is equivalent to mcpwm_set_signal_low()
 * @param duty float -> duty (percent value (0-100) of PWM duty Cycle)
 * Input Voltage: 24V 
 * Output Voltage = 24 x Duty 
 * Ex: duty = 50, Vout = 24V * .50 = 12V
 * duty MAX value: 100, MIN 40
 */
void vL298N_control(float duty)
{
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty));
    vTaskDelay(pdMS_TO_TICKS(100));
}

/**
 * @brief 
 * Suspend PWM Unit
 */
void vSuspend_PWM(void)
{
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
}


