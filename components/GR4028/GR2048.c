static volatile uint16_t flow_samples;
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