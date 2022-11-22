#include "driver/adc.h"

#define NO_OF_SAMPLES CONFIG_NO_OF_SAMPLES
#define DEFAULT_VREF 1500        // ADCout = (Vin, ADC*2^12)/Vref

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;   // GPIO34 
static const adc_bits_width_t width = ADC_WIDTH_12Bit;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static esp_err_t ADC_init();

static void Temp_Sense();