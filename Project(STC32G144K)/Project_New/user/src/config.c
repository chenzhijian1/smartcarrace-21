#include "config.h"
#include "car_control.h"
#include "element.h"
#include "huandao.h"
#include "inductance.h"

#define CONFIG_FLASH_ADDR             (0x0000UL)
#define CONFIG_FLASH_PAGE_SIZE        (512U)
#define CONFIG_FLASH_MAGIC            (0x43464731UL)
#define CONFIG_FLASH_VERSION          (5U)
#define CONFIG_FLASH_VERSION_V4       (4U)

#define CONFIG_BUTTON_PIN             P35
#define CONFIG_BUTTON_LONG_SAMPLES    (100U)
#define CONFIG_BUTTON_SAMPLE_MS       (10U)

typedef struct
{
    uint16 crc;
    uint32 magic;
    uint16 version;
    uint16 size;

    float kpa;
    float kpb;
    float kd;
    float kd_imu;
    float kp_motor;
    float ki_motor;
    int16 normal_speed;

    float inductance_a;
    float inductance_b;
    float inductance_c;

    float distance_before_huandao[HUANDAO_MAX_COUNT];
    uint8 element_lap_target;
    uint16 suction_fan_pwm_start;
    uint8 element_reverse_run;
} config_flash_t;

typedef struct
{
    uint16 crc;
    uint32 magic;
    uint16 version;
    uint16 size;

    float kpa;
    float kpb;
    float kd;
    float kd_imu;
    float kp_motor;
    float ki_motor;
    int16 normal_speed;

    float inductance_a;
    float inductance_b;
    float inductance_c;

    float distance_before_huandao[HUANDAO_MAX_COUNT];
    uint8 element_lap_target;
    uint16 suction_fan_pwm_start;
} config_flash_v4_t;

typedef char config_flash_size_check[
    (sizeof(config_flash_t) <= CONFIG_FLASH_PAGE_SIZE) ? 1 : -1];
typedef char config_flash_v4_size_check[
    (sizeof(config_flash_v4_t) <= CONFIG_FLASH_PAGE_SIZE) ? 1 : -1];

uint8 debug_mode = 0;

float kpa = 50.0f;
float kpb = 80.0f;
float kd = 70.0f;
float kd_imu = 10.0f;

float kp_motor = 10.0f;
float ki_motor = 2.0f;
float kd_motor = 0.0f;

float speed_high = 500.0f;
float speed_low = 400.0f;
float speed_90 = 300.0f;
float speed_S = 200.0f;
int16 normal_speed = 0;

static config_flash_t flash_config;
static config_flash_t flash_verify;
static int16 configured_normal_speed = 0;

static uint16 Config_Crc16(const uint8 *buffer, uint16 len)
{
    uint16 crc = 0xFFFFU;
    uint8 bit_index;

    while (len--)
    {
        crc ^= (uint16)(*buffer++) << 8;
        for (bit_index = 0; bit_index < 8; bit_index++)
        {
            if (crc & 0x8000U)
                crc = (uint16)((crc << 1) ^ 0x1021U);
            else
                crc <<= 1;
        }
    }

    return crc;
}

static uint16 Config_RecordCrc(const config_flash_t *record)
{
    const uint8 *bytes = (const uint8 *)record;

    return Config_Crc16(bytes + sizeof(record->crc),
                        (uint16)(sizeof(*record) - sizeof(record->crc)));
}

static uint16 Config_RecordV4Crc(const config_flash_v4_t *record)
{
    const uint8 *bytes = (const uint8 *)record;

    return Config_Crc16(bytes + sizeof(record->crc),
                        (uint16)(sizeof(*record) - sizeof(record->crc)));
}

static uint8 Config_RecordIsValid(const config_flash_t *record)
{
    uint8 i;

    if (record->magic != CONFIG_FLASH_MAGIC ||
        record->version != CONFIG_FLASH_VERSION ||
        record->size != sizeof(*record) ||
        record->crc != Config_RecordCrc(record))
    {
        return 0;
    }

    if (record->element_lap_target == 0 ||
        record->element_reverse_run > 1U ||
        record->normal_speed < 0 ||
        record->suction_fan_pwm_start > MOTOR_PWM_MAX)
    {
        return 0;
    }

    for (i = 0; i < HUANDAO_MAX_COUNT; i++)
    {
        if (record->distance_before_huandao[i] < 0.0f)
        {
            return 0;
        }
    }

    return 1;
}

static uint8 Config_RecordV4IsValid(const config_flash_v4_t *record)
{
    uint8 i;

    if (record->magic != CONFIG_FLASH_MAGIC ||
        record->version != CONFIG_FLASH_VERSION_V4 ||
        record->size != sizeof(*record) ||
        record->crc != Config_RecordV4Crc(record))
    {
        return 0;
    }

    if (record->element_lap_target == 0 ||
        record->normal_speed < 0 ||
        record->suction_fan_pwm_start > MOTOR_PWM_MAX)
    {
        return 0;
    }

    for (i = 0; i < HUANDAO_MAX_COUNT; i++)
    {
        if (record->distance_before_huandao[i] < 0.0f)
            return 0;
    }

    return 1;
}

static void Config_Capture(config_flash_t *record)
{
    uint8 i;

    memset(record, 0, sizeof(*record));
    record->magic = CONFIG_FLASH_MAGIC;
    record->version = CONFIG_FLASH_VERSION;
    record->size = sizeof(*record);

    record->kpa = kpa;
    record->kpb = kpb;
    record->kd = kd;
    record->kd_imu = kd_imu;
    record->kp_motor = kp_motor;
    record->ki_motor = ki_motor;
    record->normal_speed = configured_normal_speed;

    record->inductance_a = A_;
    record->inductance_b = B_;
    record->inductance_c = C_;

    for (i = 0; i < HUANDAO_MAX_COUNT; i++)
        record->distance_before_huandao[i] = distance_before_huandao[i];
    record->element_lap_target = element_lap_target;
    record->suction_fan_pwm_start = suction_fan_pwm_start;
    record->element_reverse_run = element_reverse_run;
    record->crc = Config_RecordCrc(record);
}

static void Config_Apply(const config_flash_t *record)
{
    uint8 i;

    kpa = record->kpa;
    kpb = record->kpb;
    kd = record->kd;
    kd_imu = record->kd_imu;
    kp_motor = record->kp_motor;
    ki_motor = record->ki_motor;
    configured_normal_speed = record->normal_speed;

    A_ = record->inductance_a;
    B_ = record->inductance_b;
    C_ = record->inductance_c;

    for (i = 0; i < HUANDAO_MAX_COUNT; i++)
        distance_before_huandao[i] = record->distance_before_huandao[i];
    element_lap_target = record->element_lap_target;
    suction_fan_pwm_start = record->suction_fan_pwm_start;
    element_reverse_run = record->element_reverse_run;

    motor_left.Kp_motor = kp_motor;
    motor_left.Ki_motor = ki_motor;
    motor_right.Kp_motor = kp_motor;
    motor_right.Ki_motor = ki_motor;
}

static void Config_ApplyV4(const config_flash_v4_t *record)
{
    uint8 i;

    kpa = record->kpa;
    kpb = record->kpb;
    kd = record->kd;
    kd_imu = record->kd_imu;
    kp_motor = record->kp_motor;
    ki_motor = record->ki_motor;
    configured_normal_speed = record->normal_speed;

    A_ = record->inductance_a;
    B_ = record->inductance_b;
    C_ = record->inductance_c;

    for (i = 0; i < HUANDAO_MAX_COUNT; i++)
        distance_before_huandao[i] = record->distance_before_huandao[i];
    element_lap_target = record->element_lap_target;
    suction_fan_pwm_start = record->suction_fan_pwm_start;
    element_reverse_run = 0;

    motor_left.Kp_motor = kp_motor;
    motor_left.Ki_motor = ki_motor;
    motor_right.Kp_motor = kp_motor;
    motor_right.Ki_motor = ki_motor;
}

void Config_Init(void)
{
    iap_init();
    memset(&flash_config, 0, sizeof(flash_config));
    iap_read_buff(CONFIG_FLASH_ADDR, (uint8 *)&flash_config,
                  (uint16)sizeof(flash_config));

    if (Config_RecordIsValid(&flash_config))
    {
        Config_Apply(&flash_config);
        printf("config,load_ok,v%u\r\n", CONFIG_FLASH_VERSION);
    }
    else
    {
        memset(&flash_config, 0, sizeof(flash_config));
        iap_read_buff(CONFIG_FLASH_ADDR, (uint8 *)&flash_config,
                      (uint16)sizeof(config_flash_v4_t));
        if (Config_RecordV4IsValid(
                (const config_flash_v4_t *)&flash_config))
        {
            Config_ApplyV4((const config_flash_v4_t *)&flash_config);
            printf("config,load_ok,v4,reverse_default_0\r\n");
        }
        else
        {
            configured_normal_speed = 0;
            element_reverse_run = 0;
            printf("config,default\r\n");
        }
    }
}

uint8 Config_Save(void)
{
    Config_Capture(&flash_config);

    iap_erase_page(CONFIG_FLASH_ADDR);
    iap_write_buff(CONFIG_FLASH_ADDR, (uint8 *)&flash_config,
                   (uint16)sizeof(flash_config));

    memset(&flash_verify, 0, sizeof(flash_verify));
    iap_read_buff(CONFIG_FLASH_ADDR, (uint8 *)&flash_verify,
                  (uint16)sizeof(flash_verify));

    if (!Config_RecordIsValid(&flash_verify) ||
        memcmp(&flash_config, &flash_verify, sizeof(flash_config)) != 0)
    {
        return 0;
    }

    return 1;
}

static void Config_PrintAll(void)
{
    uint8 i;

    printf("CONFIG_BEGIN,v%u\r\n", CONFIG_FLASH_VERSION);
    printf("kpa=%.2f,kpb=%.2f,kd=%.2f,kd_imu=%.2f\r\n",
           kpa, kpb, kd, kd_imu);
    printf("kp_motor=%.2f,ki_motor=%.2f\r\n", kp_motor, ki_motor);
    printf("normal_speed=%d\r\n", configured_normal_speed);
    printf("A=%.3f,B=%.3f,C=%.3f\r\n", A_, B_, C_);
    for (i = 0; i < HUANDAO_MAX_COUNT; i++)
    {
        printf("distance_before_huandao[%u]=%.1f\r\n",
               i, distance_before_huandao[i]);
    }
    printf("element_lap_target=%u\r\n", element_lap_target);
    printf("element_reverse_run=%u\r\n", element_reverse_run);
    printf("suction_fan_pwm_start=%u\r\n", suction_fan_pwm_start);
    printf("CONFIG_END\r\n");
}

void Config_SetNormalSpeed(int16 speed)
{
    configured_normal_speed = speed;
    normal_speed = speed;
}

void Config_ButtonInit(void)
{
    gpio_init(CONFIG_BUTTON_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

void Config_ButtonPoll(void)
{
    uint16 hold_samples = 0;

    if (normal_speed != 0 || CONFIG_BUTTON_PIN != 0)
        return;

    system_delay_ms(20);
    if (CONFIG_BUTTON_PIN != 0)
        return;

    while (CONFIG_BUTTON_PIN == 0 &&
           hold_samples < CONFIG_BUTTON_LONG_SAMPLES)
    {
        system_delay_ms(CONFIG_BUTTON_SAMPLE_MS);
        hold_samples++;
    }

    if (hold_samples >= CONFIG_BUTTON_LONG_SAMPLES)
    {
        debug_mode = (uint8)!debug_mode;
        while (CONFIG_BUTTON_PIN == 0)
            system_delay_ms(CONFIG_BUTTON_SAMPLE_MS);
        return;
    }

    if (debug_mode)
    {
        if (Config_Save())
        {
            Config_PrintAll();
        }
    }
    else
    {
        Config_Init();
        Element_Init();
        if (configured_normal_speed > 0)
        {
            normal_speed = configured_normal_speed;
            printf("launch,config_ok,speed,%d\r\n", normal_speed);
        }
        else
        {
            printf("launch,blocked,speed_zero\r\n");
        }
    }
}

/* Legacy ASCII helpers remain only for navigation path storage. */
float StrToDouble(const char *s)
{
    int i = 0;
    int k = 0;
    float j;
    int symbol = 1;
    float result = 0.0f;

    if (s[i] == '+')
        i++;
    if (s[i] == '-')
    {
        i++;
        symbol = -1;
    }
    while (s[i] != '\0' && s[i] != '.')
    {
        j = (s[i] - '0') * 1.0f;
        result = result * 10.0f + j;
        i++;
    }
    if (s[i] == '.')
    {
        i++;
        while (s[i] != '\0' && s[i] != ' ')
        {
            k++;
            j = s[i] - '0';
            result += j / (float)pow(10.0, k);
            i++;
        }
    }
    return symbol * result;
}

float Config_ReadFloat(uint8 len, uint16 addr)
{
    uint8 buf[34];

    if (len >= sizeof(buf))
        len = (uint8)(sizeof(buf) - 1U);
    memset(buf, 0, sizeof(buf));
    iap_read_buff(addr, buf, len);
    buf[len] = '\0';
    return StrToDouble((const char *)buf);
}

void eeprom_write_float_ascii(double dat, uint8 num, uint8 pointnum, uint16 addr)
{
    uint32 length;
    int8 buff[34];
    int8 start;
    int8 end;
    int8 point;

    memset(buff, 0, sizeof(buff));
    if (dat < 0)
        length = sprintf((char *)buff, "%f", dat);
    else
    {
        length = sprintf((char *)&buff[1], "%f", dat);
        length++;
    }

    point = (int8)(length - 7);
    start = (int8)(point - num - 1);
    end = (int8)(point + pointnum + 1);
    while (start < 0)
    {
        buff[end++] = ' ';
        start++;
    }
    buff[start] = (dat < 0) ? '-' : '+';
    buff[end - 1] = '\0';
    buff[end] = '\n';
    extern_iap_write_buff(addr, (uint8 *)buff,
                          (uint16)(num + pointnum + 3));
}
