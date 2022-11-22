/**
 * @file    esp_demo.c
 * @author  Dmitri Lyalikov
 * @brief   7x24 Application Source for ESP32 
 * @version 1.1
 * @date    2022-10-26
 * @copyright Copyright (c) 2022
 * 
 * Entry Point and Application, App_Main():
 *     Configures Hardware, ADC, L298N modules via init() functions
 *     Creates Resource sharing mechanisms (xSenseQueue, xFlowQueue) and Mutex
 *     Assign tasks to Cores, Networking performed on Core 0, 
 *     Hardware interfacing and Control tasks on Core 1
 * 
 *     ESP32 PIN TOPOLOGY
 *     -GPIO34 <-> LM35 Vout
 *     -GPIO32 <-> L298N IN1_0
 *     -GPIO33 <-> L298N IN2_0
 *     -GPIO25 <-> L298N EN1_0
 *     -GPIO27 <-> L298N IN1_1
 *     -GPIO14 <-> L298N IN2_1
 *     -GPIO12 <-> L298N EN1_1
 *     -GPIO21 <-> G 1/4 Flow Sensor
 */
#include <stdio.h>
#include <sys/param.h>
#include <stdbool.h>
#include <string.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "perfmon.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "L298N.h"

#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include <esp_event.h>
#include "esp_wpa2.h"
#include "esp_netif.h"

#include "smbus.h"
#include "i2c-lcd1602.h"



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

#define WIFI_SSID "JasperNet"
#define EAP_METHOD CONFIG_EAP_METHOD

#define EAP_ID "Your_UPI/UserID"
#define EAP_USERNAME "Your_UPI/UserID"
#define EAP_PASSWORD "Password"

#define ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define FLOW_RATE_PIN  GPIO_NUM_21

#define DEFAULT_VREF 1500        // ADCout = (Vin, ADC*2^12)/Vref
#define NO_OF_SAMPLES 20

#define MIN_INTEGRAL 2.0         // Min integral value
#define MAX_INTEGRAL 10.0        // Max Integral Value
#define EPSILON      0.01

static const char* TAG = "7x24 APP";

static const float kp = 2;       //Integral Ratio
static const float ki = 5;       // Differential ratio 1
static const float kd = 1;       // Sum of errors
static volatile float sum = 0;
static volatile float old_error = 0;

static float set_temp = 20;      // Desired Temperature of Target (Celsius)
static volatile uint16_t flow_samples;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;   // GPIO34 
static const adc_bits_width_t width = ADC_WIDTH_12Bit;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static SemaphoreHandle_t xQueueMutex;
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

typedef struct xSense_t
{
    TickType_t xTimeStamp;
    uint16_t ulValue;
}xSense_t;

// Global resource queue handles 
QueueHandle_t xSense_Queue, xFlow_Queue, xgpio_evt;

/**
 * @brief Initialize Temperature and Flow Rate Sense Mailbox Queue
 */
static QueueHandle_t vQueueInit(void)
{   
    return xQueueCreate(1, sizeof(xSense_t));
}

void vUpdateQueue(QueueHandle_t Queue, float ulNewValue)
{
    xSense_t xData;
    xData.ulValue = ulNewValue;
    xData.xTimeStamp = pdTICKS_TO_MS(xTaskGetTickCount());
    // xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(1000));
    xQueueOverwrite(Queue, &xData);
    // xSemaphoreGive(xQueueMutex);
}

/**
 * @brief 
 *        Read Mailbox Queue
 * @param pxData Pointer to Struct of type xSense_t for contents to be copied
 * @param Queue  Pointer to struct of type xSense_T to be read from
 */
static void vReadQueue(xSense_t *pxData, QueueHandle_t Queue)
{
    xQueuePeek(Queue, pxData, portMAX_DELAY);
}

static void IRAM_ATTR vFlow_ISR_Handler(void* arg)
{
    flow_samples++;
}
/**
 * @brief Flow Rate Task to Read From GR-4028
 *        Writes to xFlow_Queue every 60 seconds
 * Flow_Rate = (Pulse Frequency) / 38 (Flow Rate in liters per minute)
 */
static void vFlow_Rate_Task(void *pvParameter)
{
    uint32_t flow_rate = 0;
    for (;;){
    flow_rate = ((flow_samples / 38) / 10 ) + 4 ;
    flow_rate = 0; 
    vUpdateQueue(xFlow_Queue, flow_rate);
    printf("Flow Rate = %d mL/s\n", flow_rate);
    flow_samples = 0;
    printf("got here\n");
    vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

/**
 * @brief Initialize GPIO 21 as Input for Flow Rate Sensor
 *        Config vFlowInterrupt_Handler as ISR to increment 
 *        flow_samples
 */
void vInit_Flow(void)
{
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = GPIO_SEL_21;
    gpioConfig.mode         = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpioConfig.intr_type    = GPIO_INTR_POSEDGE;
    gpio_config(&gpioConfig);
    
    xTaskCreatePinnedToCore(vFlow_Rate_Task,
                            "FLOW_SENSE",
                            2000,
                            NULL,
                            1,
                            NULL,
                            1); 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_RATE_PIN, vFlow_ISR_Handler, (void*) FLOW_RATE_PIN);
}


/**
 * @brief Initialize I2C Master bus for LCD1602 Display
 *        SCL - GPIO19
 *        SDA - GPIO18
 */
static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

/**
 * @brief LCD1602 Task 
 * Display Temperature and Flow Rate
 * Update every 10 seconds
 */
static void lcd1602_display()
{
    // Init I2C Master
    i2c_master_init();
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
    char temp_read[50];
    xSense_t pxData = {
        .ulValue = 0,
        .xTimeStamp = 0,
    };
    for (;;)
    {
        xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(2000));
        vReadQueue(&pxData, xSense_Queue);
        xSemaphoreGive(xQueueMutex);
        sprintf(temp_read, "%d", pxData.ulValue);
        i2c_lcd1602_move_cursor(lcd_info, 9, 0);
        i2c_lcd1602_write_string(lcd_info, temp_read);
        i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DEGREE);

        xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(2000));
        vReadQueue(&pxData, xFlow_Queue);
        xSemaphoreGive(xQueueMutex);
        sprintf(temp_read, "%d", pxData.ulValue);
        i2c_lcd1602_move_cursor(lcd_info, 10, 1);
        i2c_lcd1602_write_string(lcd_info, temp_read);

        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
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
    uint16_t temp, sum = 0;
    for (uint8_t i = 0; i < NO_OF_SAMPLES; i++){
        temp = (adc1_get_raw((adc1_channel_t)channel));    
        temp = (esp_adc_cal_raw_to_voltage(temp, adc_chars));
        sum  += temp * .1;
    }
    temp = (sum / NO_OF_SAMPLES);
    ESP_LOGI(TAG, "Read: %d", temp);
    
    xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(1000));
    // Update xSense_Queue
    vUpdateQueue(xSense_Queue, temp); 
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
        vReadQueue(&pxData, xSense_Queue);
        xSemaphoreGive(xQueueMutex);
        TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        TickType_t time_change = (now - pxData.xTimeStamp) / 1000;
        float error = -(set_temp - pxData.ulValue);
        printf("Read Temp: %d\n", pxData.ulValue);
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
        vL298N_control(100);
        printf("PID OUTPUT: %d\n", 100);
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

    nvs_flash_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(ADC_init());
    ESP_LOGI(TAG, "ADC Init OK");
    ESP_ERROR_CHECK(L298N_init());
    ESP_LOGI(TAG, "L298N Motor Control Module Init OK");

    xQueueMutex = xSemaphoreCreateMutex();
    xSense_Queue = vQueueInit();
    xFlow_Queue = vQueueInit();

    vInit_Flow();
    xTaskCreatePinnedToCore(Temp_Sense,
                            "TEMP_SENSE",
                            1000,
                            NULL,
                            1, 
                            NULL,
                            1);

    xTaskCreatePinnedToCore(&vPIDCompute,
                            "PID_Compute",
                            1750,
                            NULL,
                            1,
                            NULL,
                            1); 

    xTaskCreatePinnedToCore(&lcd1602_display,
                            "LCD_Display",
                            3000,
                            NULL,
                            1,
                            NULL,
                            1);  
}
