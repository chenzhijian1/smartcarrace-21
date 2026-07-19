#include "my_peripheral.h"
#include "car_control.h"
#include "element.h"

/*============================================================================
 * æ¨¡å—è¯´æ˜Žï¼šå¤–è®¾æŽ§åˆ¶æ¨¡å?
 * åŠŸèƒ½ï¼šèœ‚é¸£å™¨ã€é™€èžºä»ªå¤„ç†ã€TOFä¼ æ„Ÿå™?
 * æ³¨æ„ï¼šé™€èžºä»ªå§¿æ€è§£ç®—å·²ç§»è‡³quaternion.cæ¨¡å—
 *============================================================================*/

/*---------------------------------------------------------------------------
 * é™€èžºä»ªç›¸å…³å˜é‡
 *---------------------------------------------------------------------------*/
volatile uint8 flag_gyro_z = 0;
volatile uint8 gyro_update_ticks = 0;

/*---------------------------------------------------------------------------
 * TOFä¼ æ„Ÿå™¨å˜é‡?
 *---------------------------------------------------------------------------*/
uint16 tof[2] = {0};
// uint8 count_tof = 0;
uint8 count_flag = 0;
static uint8 uart_cmd_buf[50];
static uint8 uart_cmd_rx_data[16];
static uint8 uart_cmd_index = 0;
volatile uint16 uart_feedback_hold_ticks = 0;
extern uint8 send_flag;

static void uart_feedback_hold_start(void)
{
    uart_feedback_hold_ticks = 400;
    send_flag = 0;
}

static void uart_command_print_params(void)
{
    printf("%.2f,%.2f,", kpa, kpb);
    printf("%.2f,%.2f,", kd, kd_imu);
    printf("%.2f,%.2f,", kp_motor, ki_motor);
    printf("%d,", normal_speed);
    printf("%.2f,", s);
    printf("%.2f,%.2f,", A_, B_);
    printf("%.2f\r\n", C_);
    uart_feedback_hold_start();
}

static float uart_cmd_to_float(const char *str)
{
    uint8 i;
    float result = 0.0f;
    float sign = 1.0f;
    int decimal_place = 0;
    int is_fraction = 0;

    if (*str == '-')
    {
        sign = -1.0f;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }

    while (*str)
    {
        if (*str == '.')
        {
            is_fraction = 1;
            str++;
            continue;
        }

        if (*str < '0' || *str > '9')
        {
            break;
        }

        if (is_fraction)
        {
            decimal_place++;
        }

        result = result * 10.0f + (*str - '0');
        str++;
    }

    for (i = 0; i < decimal_place; i++)
    {
        result /= 10.0f;
    }

    return result * sign;
}

static uint8 uart_command_apply(char *cmd)
{
    float value;
    uint8 applied = 1;

    // t0~t3: Ñ¡Ôñ´®¿ÚÊä³öÄ£Ê½£¬¶ÔÓ¦ main.c ÖÐ uart_telemetry_print µÄ case 0~3
    if (cmd[0] == 't' && cmd[1] >= '0' && cmd[1] <= '3' && cmd[2] == '\0')
    {
        uart_output_mode = (uint8)(cmd[1] - '0');
        printf("mode,%d\r\n", uart_output_mode);
        uart_feedback_hold_start();
        return 1;
    }

    // f0~f9: Ö±½ÓÐÞ¸Ä³µÁ¾×´Ì¬ flag£¬ÀýÈç f4 ½øÈëÆð²½×´Ì¬£¬f5 ½øÈëÂýËÙÍ£³µ×´Ì¬
    if (cmd[0] == 'f' && cmd[1] >= '0' && cmd[1] <= '9' && cmd[2] == '\0')
    {
        flag = (uint8)(cmd[1] - '0');
        printf("flag,%d\r\n", flag);
        uart_feedback_hold_start();
        return 1;
    }

    if (cmd[0] == '\0' || cmd[1] == '\0')
    {
        return 0;
    }

    value = uart_cmd_to_float((const char *)(cmd + 1));

    switch (cmd[0])
    {
        case 'a': kpa = value; break;       // a+ÊýÖµ: ÐÞ¸Ä·½Ïò»·±ÈÀýÏµÊý kpa
        case 'b': kpb = value; break;       // b+ÊýÖµ: ÐÞ¸Ä·½Ïò»··ÇÏßÐÔÏµÊý kpb
        case 'd': kd = value; break;        // d+ÊýÖµ: ÐÞ¸Ä·½Ïò»·Î¢·ÖÏµÊý kd
        case 'D': kd_imu = value; break;    // D+ÊýÖµ: ÐÞ¸ÄÍÓÂÝÒÇ½ÇËÙ¶È·´À¡ÏµÊý kd_imu

        case 'p':                           // p+ÊýÖµ: ÐÞ¸ÄËÙ¶È»· P£¬²¢Í¬²½µ½×óÓÒµç»ú
            kp_motor = value;
            motor_left.Kp_motor = kp_motor;
            motor_right.Kp_motor = kp_motor;
            break;

        case 'i':                           // i+ÊýÖµ: ÐÞ¸ÄËÙ¶È»· I£¬²¢Í¬²½µ½×óÓÒµç»ú
            ki_motor = value;
            motor_left.Ki_motor = ki_motor;
            motor_right.Ki_motor = ki_motor;
            break;

        case 'n':                           // n+ÊýÖµ: ÐÞ¸ÄÄ¿±êËÙ¶È normal_speed£¬n0 ´¥·¢ÈíÍ£³µ
            if ((int16)value == 0)
                CarControl_RequestSoftStop();
            else
                normal_speed = (int16)value;
            break;

        case 'u':                           // u+ÊýÖµ: ÐÞ¸Ä¸ºÑ¹·çÉÈÆô¶¯Ä¿±ê PWM£¬Õ¼¿Õ±È·¶Î§ 0~10000
            suction_fan_pwm_start = (uint16)motor_pwm_limit((int)value);
            printf("fan_start,%d\r\n", suction_fan_pwm_start);
            uart_feedback_hold_start();
            return 1;

        case 'v':
            suction_fan_pwm_cylinder = (uint16)motor_pwm_limit((int)value);
            printf("fan_cylinder,%d\r\n", suction_fan_pwm_cylinder);
            uart_feedback_hold_start();
            return 1;

        case 's': s = value; break;         // s+ÊýÖµ: ÐÞ¸ÄÍäµÀËÙ¶ÈË¥¼õÏµÊý s
        case 'A': A_ = value; break;        // A+ÊýÖµ: ÐÞ¸Äµç¸ÐÎó²î¹«Ê½ÏµÊý A_
        case 'B': B_ = value; break;        // B+ÊýÖµ: ÐÞ¸Äµç¸ÐÎó²î¹«Ê½ÏµÊý B_
        case 'C': C_ = value; break;        // C+ÊýÖµ: ÐÞ¸Äµç¸ÐÎó²î¹«Ê½ÏµÊý C_
        default: applied = 0; break;        // Î´Ê¶±ðÃüÁî: ²»ÐÞ¸Ä²ÎÊý£¬Ò²²»»ØÏÔ²ÎÊý±í
    }

    if (applied)
    {
        uart_command_print_params();
    }

    return applied;
}

/*---------------------------------------------------------------------------
 * å®šæ—¶å™¨ä¸­æ–­å›žè°?IMUæ•°æ®æ›´æ–°)
 * è¯´æ˜Žï¼šä½¿ç”¨å››å…ƒæ•°ç®—æ³•è¿›è¡Œå§¿æ€è§£ç®—ï¼Œæ›´æ–°euler.yawç­‰æ¬§æ‹‰è§’
 *---------------------------------------------------------------------------*/
void pit_callback(void) {
    flag_gyro_z = 1;
    if (gyro_update_ticks < 200)
    {
        gyro_update_ticks++;
    }
}

void uart_command_poll(void)
{
    uint32 len;
    uint32 i;

    len = wireless_uart_read_buffer(uart_cmd_rx_data, sizeof(uart_cmd_rx_data));
    if (len == 0)
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        if (uart_cmd_index == 1 && uart_cmd_buf[0] == 't' && uart_cmd_rx_data[i] >= '0' && uart_cmd_rx_data[i] <= '3')
        {
            uart_cmd_buf[1] = uart_cmd_rx_data[i];
            uart_cmd_buf[2] = '\0';
            uart_command_apply((char *)uart_cmd_buf);
            uart_cmd_index = 0;
        }
        else if (uart_cmd_index == 1 && uart_cmd_buf[0] == 'f' && uart_cmd_rx_data[i] >= '0' && uart_cmd_rx_data[i] <= '9')
        {
            uart_cmd_buf[1] = uart_cmd_rx_data[i];
            uart_cmd_buf[2] = '\0';
            uart_command_apply((char *)uart_cmd_buf);
            uart_cmd_index = 0;
        }
        else if (uart_cmd_rx_data[i] == '\r' || uart_cmd_rx_data[i] == '\n')
        {
            uart_cmd_buf[uart_cmd_index] = '\0';
            uart_command_apply((char *)uart_cmd_buf);
            uart_cmd_index = 0;
        }
        else if (uart_cmd_index < (sizeof(uart_cmd_buf) - 1))
        {
            uart_cmd_buf[uart_cmd_index++] = uart_cmd_rx_data[i];
        }
        else
        {
            uart_cmd_index = 0;
        }
    }
}

/*---------------------------------------------------------------------------
 * èœ‚é¸£å™¨åˆå§‹åŒ–
 *---------------------------------------------------------------------------*/
void beep_init(void) {
    gpio_init(IO_P67, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    BEEP = 1;
}

/*---------------------------------------------------------------------------
 * èœ‚é¸£å™¨æµ‹è¯?0.5sé—´éš”)
 *---------------------------------------------------------------------------*/
void beep_test(void) {
    BEEP = !BEEP;
    system_delay_ms(500);
}

/*---------------------------------------------------------------------------
 * éšœç¢ç‰©åˆ¤æ–?æš‚ç•™ç©ºï¼Œæ ¹æ®éœ€è¦å®žçŽ?
 *---------------------------------------------------------------------------*/
void block_judgement(void) {
    
}


