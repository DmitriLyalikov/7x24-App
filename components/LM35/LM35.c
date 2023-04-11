
#include "LM35.h"

static const char* TAG = "Temperature {LM35}";

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;   // GPIO34 
static const adc_bits_width_t width = ADC_WIDTH_12Bit;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static const adc_channel_t ts1_channel = ADC_CHANNEL_4; // GPIO 32
static const adc_channel_t ts2_channel = ADC_CHANNEL_5; // GPIO 33


static QueueHandle_t xSenseQueue;
static SemaphoreHandle_t xQueueMutex;

/**
*@brief
*  Config ADC 
*  Channel 6 Unit 1 
*  (12 Bit Resolution, 0 Attenuation, 1V2 Reference)
*/
void vInit_TempSense_Task(QueueHandle_t xSenseQueue, SemaphoreHandle_t xQueueMutex)
{
    adc1_config_channel_atten((adc1_channel_t)channel, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    ESP_LOGI(TAG, "Starting ADC/Temp Sense Task");
    xTaskCreatePinnedToCore(vTempSense_Task,
                            "TEMP_SENSE",
                            1000,
                            NULL,
                            1, 
                            NULL,
                            1);
}   


/**
*@brief Temperature Sense Task
*  Read Temperature from LM35 Over ADC 
*  LM35 Transfer Function: Vout = 10mV/C *T
*  Where T is temperature in Celsius
*  Task Repeats every 1000ms
*  Take (NO_OF_SAMPLES) of reads and use average
*  Update xSense_Queue mailbox temp
*/
void vTempSense_Task(void *pvParameters)
{   
    for(;;){
    uint16_t temp, sum = 0;
    for (uint8_t i = 0; i < NO_OF_SAMPLES; i++){
        temp = (adc1_get_raw((adc1_channel_t)channel));    
        temp = (esp_adc_cal_raw_to_voltage(temp, adc_chars));
        sum  += temp * .1;
    }
    temp = (sum / NO_OF_SAMPLES);
    ESP_LOGI(TAG, "Temperature Read Celsius: %d", temp);
    
    xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(1000));
    // Update xSense_Queue
    vUpdateQueue(xSenseQueue, TAG, temp); 
    xSemaphoreGive(xQueueMutex);
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD));
    }
}