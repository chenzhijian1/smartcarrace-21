#include "inductance.h"
#include "huandao.h"
#include "zf_driver_adc.h"

#define EM_ADC_CHANNEL ADC1_CH0_P10
#define EM_ADC_DISCHARGE_PIN IO_P11

#define EM_MUX1_PIN IO_P03
#define EM_OE1_PIN IO_P04
#define EM_MUX2_PIN IO_P01
#define EM_OE2_PIN IO_P02
#define EM_RMUX_PIN IO_P46
#define EM_ROE_PIN IO_P00

#define EM_SWITCH_SETTLE_US (20)
#define EM_PEAK_BUILD_US (80)
#define EM_DISCHARGE_US (20)
#define EM_ADC_DUMMY_COUNT (1)
#define EM_ADC_SAMPLE_COUNT (4)

uint8 flag1 = 0;
volatile uint8 flag_adc = 0;

float A_ = 1.0f;
float B_ = 1.4f;
float C_ = 1.0f;

adc_struct aaddcc = {0};
const uint16 MAX_ADC[NUM] = {4095, 4095, 4095, 4095, 4095};
uint16 AD_value[NUM][10];
uint16 ad_ave[NUM] = {0};
float AD_ONE[NUM] = {0};
float adc_left_dir = 0.0;
float adc_right_dir = 0.0;

static void em_all_signal_mux_off(void)
{
    gpio_high(EM_OE1_PIN);
    gpio_high(EM_OE2_PIN);
}

static void em_adc_pin_to_discharge(void)
{
    gpio_init(EM_ADC_DISCHARGE_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

static void em_adc_pin_release(void)
{
    gpio_init(EM_ADC_DISCHARGE_PIN, GPI, GPIO_HIGH, GPI_IMPEDANCE);
}

static void em_discharge_peak_cap(void)
{
    em_adc_pin_to_discharge();
    system_delay_us(EM_DISCHARGE_US);
    em_adc_pin_release();
}

static void em_set_gain_20p7x(void)
{
    gpio_high(EM_RMUX_PIN);
    gpio_low(EM_ROE_PIN);
    system_delay_us(EM_SWITCH_SETTLE_US);
}

static void em_select_channel(uint8 channel)
{
    em_all_signal_mux_off();

    switch (channel)
    {
    case 0:
        gpio_high(EM_MUX1_PIN);
        gpio_low(EM_OE1_PIN);
        break;

    case 1:
        gpio_low(EM_MUX1_PIN);
        gpio_low(EM_OE1_PIN);
        break;

    case 2:
        gpio_high(EM_MUX2_PIN);
        gpio_low(EM_OE2_PIN);
        break;

    case 3:
        gpio_low(EM_MUX2_PIN);
        gpio_low(EM_OE2_PIN);
        break;

    default:
        break;
    }

    system_delay_us(EM_SWITCH_SETTLE_US);
}

static uint16 em_read_channel(uint8 channel)
{
    uint8 i;
    uint32 sum = 0;

    em_discharge_peak_cap();
    em_select_channel(channel);
    system_delay_us(EM_PEAK_BUILD_US);

    for (i = 0; i < EM_ADC_DUMMY_COUNT; i++)
    {
        (void)adc_convert(EM_ADC_CHANNEL);
    }

    for (i = 0; i < EM_ADC_SAMPLE_COUNT; i++)
    {
        sum += adc_convert(EM_ADC_CHANNEL);
    }

    em_all_signal_mux_off();
    em_discharge_peak_cap();

    return (uint16)(sum / EM_ADC_SAMPLE_COUNT);
}

void direction_adc_init(void)
{
    gpio_init(EM_OE1_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(EM_OE2_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(EM_ROE_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    gpio_init(EM_MUX1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(EM_MUX2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(EM_RMUX_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    em_adc_pin_release();
    adc_init(EM_ADC_CHANNEL, ADC_12BIT);

    em_set_gain_20p7x();
    em_discharge_peak_cap();
}

void cleanADC(void)
{
    uint8 i;
    for (i = 0; i < NUM; i++)
    {
        ad_ave[i] = 0;
    }
}

void direction_adc_get(void)
{
    int i;
    float err_den;

    cleanADC();

    for (i = 0; i < 5; i++)
    {
        AD_value[0][i] = em_read_channel(0);
        AD_value[1][i] = em_read_channel(1);
        AD_value[3][i] = em_read_channel(2);
        AD_value[4][i] = em_read_channel(3);
    }

    for (i = 0; i < 5; i++)
    {
        ad_ave[0] += AD_value[0][i];
        ad_ave[1] += AD_value[1][i];
        // ad_ave[2] += AD_value[2][i];
        ad_ave[3] += AD_value[3][i];
        ad_ave[4] += AD_value[4][i];
    }

    for (i = 0; i < NUM; i++)
    {
        ad_ave[i] /= 5;
        if (ad_ave[i] > MAX_ADC[i])
        {
            ad_ave[i] = MAX_ADC[i];
        }
    }

    for (i = 0; i < NUM; i++)
    {
        AD_ONE[i] = 100 * (float)ad_ave[i] / MAX_ADC[i];
    }

    aaddcc.last_err_dir = aaddcc.err_dir;

    if (Huandao_DetectUpdate())
    {
        aaddcc.err_dir = 0.0f;
        aaddcc.last_err_dir = 0.0f;
        return;
    }

    if (AD_ONE[0] + AD_ONE[1] + AD_ONE[3] + AD_ONE[4] < 4)
    {
        aaddcc.err_dir = aaddcc.last_err_dir;
    }
    else
    {
        err_den = A_ * (AD_ONE[0] + AD_ONE[4]) + C_ * fabs(AD_ONE[1] - AD_ONE[3]);
        if (err_den > 0.001f)
        {
            aaddcc.err_dir = 30.0f * (A_ * (AD_ONE[0] - AD_ONE[4]) + B_ * (AD_ONE[1] - AD_ONE[3])) / err_den;
        }
        else
        {
            aaddcc.err_dir = aaddcc.last_err_dir;
        }
    }
}
