/**
 * @file    esp_demo.c
 * @author  Dmitri Lyalikov
 * @brief   7x24 Application Source for ESP32 
 * @version 0.1
 * @date    2022-03-07
 * @copyright Copyright (c) 2022
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
#include "nvs_flash.h"
#include "L298N.h"

#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include <esp_event.h>
#include <esp_event.h> 

// #include "certs.h"
//#include "aws_iot_config.h"
//#include "aws_iot_log.h"
//#include "aws_iot_version.h"
//#include "aws_iot_mqtt_client_interface.h"

#define WIFI_SSID "WIFI SSID"
#define WIFI_PASS "Wifi Password"

//static EventGrouphandle_t wifi_event_group; (Error unknown type)
const int CONNECTED_BIT = BIT0;

#define FLOW_RATE_PIN  GPIO_NUM_21
volatile uint16_t flow_samples;

#define DEFAULT_VREF 1500        // ADCout = (Vin, ADC*2^12)/Vref
#define NO_OF_SAMPLES 1

#define MIN_INTEGRAL 2.0         // Min integral value
#define MAX_INTEGRAL 10.0        // Max Integral Value
#define EPSILON      0.01

static const float kp = 2;       //Integral Ratio
static const float ki = 5;       // Differential ratio 1
static const float kd = 1;      // Sum of errors
static volatile float sum = 0;
static volatile float old_error = 0;

static float set_temp = 20;      // Desired Temperature of Target (Celsius)


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;   // GPIO34 
static const adc_bits_width_t width = ADC_WIDTH_12Bit;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static SemaphoreHandle_t xQueueMutex;

typedef struct xSense_t
{
    TickType_t xTimeStamp;
    float ulValue;
}xSense_t;

QueueHandle_t xSense_Queue, xFlow_Queue;


/**
 * @brief Initialize Temperature and Flow Rate Sense Mailbox Queue
 */
QueueHandle_t vQueueInit(void)
{   
    return xQueueCreate(1, sizeof(xSense_t));
}

void vUpdateQueue(QueueHandle_t Queue, float ulNewValue)
{
    xSense_t xData;
    xData.ulValue = ulNewValue;
    xData.xTimeStamp = pdTICKS_TO_MS(xTaskGetTickCount());
    xSemaphoreTake(xQueueMutex, pdMS_TO_TICKS(1000));
    xQueueOverwrite(Queue, &xData);
    xSemaphoreGive(xQueueMutex);
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

void vFlowInterrupt_Handler(void *pvParameter)
{
    flow_samples++;
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

    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_RATE_PIN, vFlowInterrupt_Handler, NULL);
}
/**
 * @brief Flow Rate Task to Read From GR-4028
 *        Writes to xFlow_Queue every 60 seconds
 * Flow_Rate = (Pulse Frequency x 60) / 38 (Flow Rate in liters per hour)
 */
static void vFlow_Rate_Task(void *pvParameter)
{
    uint32_t flow_rate = 0;
    for (;;)
    {
    flow_samples = 0;
    vTaskDelay(pdMS_TO_TICKS(60000));
    flow_rate = (flow_samples / 38 * 60); 
    vUpdateQueue(xFlow_Queue, flow_rate);
    printf("Flow Rate = %d\n", flow_rate);
    vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/*
void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data)
{
    IOT_WARN("MQTT Disconnect");
    IoT_Error_t rc = FAILURE;
  
    if(pCLient == NULL){
	return;
    }
    
    IOT_UNUSED(data);
  
    if(aws_iot_is_autoreconnect_enabled(pClient){
	IOT_INFO("Auto Reconnect is enabled, reconnecting attempt will start now");
    }
    else {
	IOT_WARN("Auto reconnect not enabled, Starting manual reconnect...");
    	rc = aws_iot_mqtt_attempt_reconnect(pClient);
	if(NETWORK_RECONNECTED ==rc) {
	    IOT_WARN("Manual reconnect successful");
	}
	else {
	    IOT_WARN("Manual reconnect failed - %d", rc);
	}
    }
}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id){
	case SYSTEM_EVENT_STA_START:
	    esp_wifi_connect();
	    break;
   	case SYSTEM_EVENT_STA_GOT_IP:
	    xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
	    break;
  	case SYSTEM_EVENT_STA_DISCONNECTED:
	    esp_wifi_connect();
	    xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
	    break;
	default:
	    break;
      }
	return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
	.sta = {
	    .ssid = WIFI_SSID,
  	    .password = WIFI_PASS,
	},
    };
    fprintf(stderr, "Setting Wifi Configuration SSID %s...\n", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK( esp_wifi_start());
}

static void record_temp_task(void *pvParameters)
{
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    fprintf(stderr, "Connected to AP\n");

    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsdefault;

    IOT_INFO("\nAWS IoT SDK Version %d.%d.%d-%d \n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);
    
    mqttInitParams.enableAutoReconnect = false; // Enable this later
    mqttInitParams.pHostURL = AWS_IOT_MQTT_HOST;
    mqttInitParms.port = AWS_IOT_MQTT_PORT;
    mqttInitParams.pRootCALocation = "";
    mqttInitParams.pDeviceCertLocation = "";
    mqttInitParams.pDevicePrivateKeyLocation = "";
    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 20000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectedHandlerData = NULL;
   
    IoT_Error_t rc = FAILURE;
    AWS_IoT_Client client;
    rc = aws_iot_mqtt_init(&client &mqttInitParams);
    if(SUCCESS != rc) {
	IOT_ERROR("aws_iot_mqtt_init returned error : %d ", rc);
      abort();
    }
  
    IoT_Client_Connect_Params = connectParams = iotClientConnectParamsDefault;
    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    connectParams.pClientID = AWS_IOT_MQTT_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t)strlen(AWS_IOT_MQTT_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    IOT_INFO("Connecting...");
    rc = aws_iot_mqtt_connect(&client, &connectParams);
    if(SUCCESS != rc){
        IOT_ERROR("(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
        abort();
    }
    
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
	IOT_ERROR("Unable to set Auto Reconnect to true - %d", rc);
      abort();
    }
    
    const char *topicName = "sdkTest/sub";
   
    char cPayload[100];
    uint32_t payloadCount = 0;
    sprintf(cPayload, "%s : %d ", " hello from SDK", payloadCount);
    
    IoT_Publish_Message_Params paramsQOS0;
    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *) cPayload;
    paramsQOS0.isRetained = 0;

    Timer sendit;
    countdown_ms(&sendit, 1500);
    
    uint32_t reconnectAttempts = 0;
    uint32_t reconnectedCount = 0;
  
    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc )) {
        IOT_DEBUG("Top of loop: payloadCount=%d, reconnectAttempt=%d, reconnectedCount=%d\n", payloadCount, reconnectAttempts, reconnectedCount);

	  // Max time the yield functioin will wait for messages
        rc = aws_iot_mqtt_yield(&client, 1000);
  	  if(NETWORK_ATTEMPTING_RECONNECT == rc) {
	    reconnectAttempts++;
	    IOT_DEBUG("Reconnecting...\n");
          // if the client is attempting to reconnect we will skip the rest of the loop
	    continue;
         }
 
   	if(NETWORK_RECONNECTED == rc) {
          reconnectedCount++;
	    IOT_DEBUG(stderr, "Reconnected...\n");
      }
    
      if(!has_timer_expired(&sendit)) {
	    IOT_INFO("--> sleeping it off");
	    vTaskDelay(1000/ portTICK_PERIOD_MS);
   	    continue;
      }
  
      sprintf(cPayload, "%s : %d ", "hello from SDK QOS0", payloadCount++);
      params.QOS0.payloadLen = strlen(cPayload);
      rc = aws_iot_mqtt_publish(&client, topicName, strlen(topicName), &paramsQOS0);
      if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
	    IOT_DEBUG("QOS0 publish ack not received.\n");
	    rc = SUCCESS;
	}
    
    if(SUCCESS != rc ) {
	IOT_ERROR("An error occurred in the loop.\n");
     }
    else {
	IOT_INFO("Publish Done\n");
    }
   
    countdown_ms(&sendit, 15000);
    }
 
    IOT_ERROR("Escaped loop...\n");
    abort();
}
*/
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
    // TickType_t xTimeNow = xTaskGetTickCount();
    float temp = 0;
    temp = (adc1_get_raw((adc1_channel_t)channel));    
    temp = (esp_adc_cal_raw_to_voltage(temp, adc_chars));
    temp = temp * .1;
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
        printf("Read Temp: %f\n", pxData.ulValue);
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
        vL298N_control(PID_Output);
        printf("PID OUTPUT: %f\n", PID_Output);
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
    printf("********ESP32 7x24 Application********\n");
    nvs_flash_init();
    printf("***Temperature Set to %f Celsius***\n", set_temp);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(ADC_init());
    printf("***LM35 Module Initialized***\n");
    ESP_ERROR_CHECK(L298N_init());
    printf("***L298N PWM Module Initialized***\n");
    xQueueMutex = xSemaphoreCreateMutex();
    xSense_Queue = vQueueInit();
    xFlow_Queue = vQueueInit();
    vInit_Flow();

    xTaskCreatePinnedToCore(Temp_Sense,
                            "TEMP_SENSE",
                            600,
                            NULL,
                            0, 
                            NULL,
                            0);
    xTaskCreatePinnedToCore(&vPIDCompute,
                            "PID_Compute",
                            1750,
                            NULL,
                            0,
                            NULL,
                            0); 
    xTaskCreatePinnedToCore(vFlow_Rate_Task,
                            "FLOW_SENSE",
                            600,
                            NULL,
                            0,
                            NULL,
                            0);
}
