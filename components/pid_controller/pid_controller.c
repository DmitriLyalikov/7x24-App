/**
 * @file L298N.c
 * @author Dmitri Lyalikov
 * @brief L298N Driver Source for Controlling DC-DC Motor Controller With PWM 
 * @version 0.1
 * @date 2022-03-02
 * @copyright Copyright (c) 2022
 * 
 */
#include "pid_controller.h"

static const char* TAG = "{PID Controller}";

static volatile float sum = 0;
static volatile float old_error = 0;

static QueueHandle_t xSense_Queue;
static SemaphoreHandle_t xQueueMutex;

void vInit_PIDCompute_Task(QueueHandle_t xSenseQueue, SemaphoreHandle_t xQueueMutex) {
    ESP_LOGI(TAG, "Starting PID Compute Task");
    xTaskCreatePinnedToCore(&vPIDCompute_Task,
                            "PID_Compute",
                            1750,
                            NULL,
                            1,
                            NULL,
                            1); 
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
void vPIDCompute_Task(void *pvParameter)
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
        vReadQueue(&pxData, xSense_Queue);
        xSemaphoreGive(xQueueMutex);
        TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        TickType_t time_change = (now - pxData.xTimeStamp) / 1000;
        float error = -( SET_TEMP - pxData.ulValue);
        // Proportional Branch
        up = PID_KP * error;
        // Integral Branch
        if (abs(error) > PID_EPSILON){
            ui = ui + error * time_change;
        }
        ui = PID_KI * ui;
        // Differential Branch
        ud = PID_KD * (error - old_error) / time_change;
        old_error = error;
        // Return PID Response
        PID_Output = up + ui + ud;
        if (PID_Output > 100){
            PID_Output = 100;
            ESP_LOGI(TAG, "Pump Max Speed Reached!");
        }
        if (PID_Output < 70){
            PID_Output = 0;
        }
        vL298N_control(PID_Output);
        ESP_LOGI(TAG, "PID OUTPUT: %f", PID_Output);
        vTaskDelay(pdMS_TO_TICKS(PID_CONTROL_PERIOD));
    }
}


