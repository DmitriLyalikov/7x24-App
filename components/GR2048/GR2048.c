#include "GR2048.h"

static const char* TAG = "Flow Sensor {GR-2048}";
static volatile uint16_t flow_samples;

void IRAM_ATTR vFlow_ISR_Handler(void* arg)
{
    flow_samples++;
}

/**
 * @brief Flow Rate Task to Read From GR-4028
 *        Writes to xFlow_Queue every 60 seconds
 * Flow_Rate = (Pulse Frequency) / 38 (Flow Rate in liters per minute)
 */
void vFlow_Rate_Task(QueueHandle_t xQueue)
{
    QueueHandle_t xFlow_Queue = xQueue;
    uint32_t flow_rate = 0;
    for (;;){
    flow_rate = ((flow_samples / 38) / 10 );
    vUpdateQueue(xFlow_Queue, flow_rate);
    ESP_LOGI(TAG, "Flow Rate = %d L/m\n", flow_rate);
    flow_samples = 0;
    vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

/**
 * @brief Initialize FLOW_RATE_PIN as Input for Flow Rate Sensor
 *        Config vFlowInterrupt_Handler as ISR to increment 
 *        flow_samples
 *        stack size 2000
 */
void vInit_Flow(void)
{
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = FLOW_RATE_PIN;
    gpioConfig.mode         = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpioConfig.intr_type    = GPIO_INTR_POSEDGE;
    gpio_config(&gpioConfig);
    ESP_LOGI(TAG, "Configured Flow Rate Pin: %d", FLOW_RATE_PIN);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_RATE_PIN, vFlow_ISR_Handler, (void*) FLOW_RATE_PIN);
    ESP_LOGI(TAG, "Registered ISR to Flow Rate Pin: %d", FLOW_RATE_PIN);
}