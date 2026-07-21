#include "headfile.h"

#define VOLTAGE_ADC_CHANNEL       ADC1_CH6_P16
#define VOLTAGE_ADC_SAMPLE_COUNT  4
#define ADC_FULL_SCALE            4095UL
#define ADC_REFERENCE_MV          2500UL
#define VOLTAGE_DIVIDER_SCALE     11UL

static uint8 voltage_adc_initialized = 0;
static volatile uint8 voltage_battery_valid = 0;
static uint16 voltage_adc_value = 0;
static uint16 voltage_p16_mv = 0;
static volatile uint16 voltage_battery_mv = 0;
static volatile uint8 voltage_battery_sample_sequence = 0;

static void voltage_adc_init(void)
{
    adc_init(VOLTAGE_ADC_CHANNEL, ADC_12BIT);
    voltage_adc_initialized = 1;
}

static void voltage_adc_get(void)
{
    uint8 i;
    uint32 sum = 0;

    if (!voltage_adc_initialized)
    {
        voltage_adc_init();
    }

    (void)adc_convert(VOLTAGE_ADC_CHANNEL);
    for (i = 0; i < VOLTAGE_ADC_SAMPLE_COUNT; i++)
    {
        sum += adc_convert(VOLTAGE_ADC_CHANNEL);
    }

    voltage_adc_value = (uint16)(sum / VOLTAGE_ADC_SAMPLE_COUNT);
    voltage_p16_mv = (uint16)((uint32)voltage_adc_value * ADC_REFERENCE_MV / ADC_FULL_SCALE);
    voltage_battery_mv = (uint16)((uint32)voltage_p16_mv * VOLTAGE_DIVIDER_SCALE);
    voltage_battery_valid = 1;
    voltage_battery_sample_sequence++;
}

uint16 voltage_battery_get_mv(void)
{
    voltage_adc_get();
    return voltage_battery_mv;
}

uint8 voltage_battery_is_low(void)
{
    static uint8 checked_sample_sequence = 0;
    static uint8 low_sample_count = 0;

    if (!voltage_battery_valid)
    {
        return 1;
    }

    if (checked_sample_sequence != voltage_battery_sample_sequence)
    {
        checked_sample_sequence = voltage_battery_sample_sequence;

        if (voltage_battery_mv < VOLTAGE_LOW_THRESHOLD_MV)
        {
            if (low_sample_count < VOLTAGE_LOW_CONFIRM_SAMPLES)
            {
                low_sample_count++;
            }
        }
        else
        {
            low_sample_count = 0;
        }
    }

    return (uint8)(low_sample_count >= VOLTAGE_LOW_CONFIRM_SAMPLES);
}
