/**
*@brief
*  Config ADC 
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