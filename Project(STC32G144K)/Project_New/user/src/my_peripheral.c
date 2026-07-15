#include "my_peripheral.h"
#include "car_control.h"

/*============================================================================
 * 模块说明：外设控制模�?
 * 功能：蜂鸣器、陀螺仪处理、TOF传感�?
 * 注意：陀螺仪姿态解算已移至quaternion.c模块
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 陀螺仪相关变量
 *---------------------------------------------------------------------------*/
volatile uint8 flag_gyro_z = 0;

/*---------------------------------------------------------------------------
 * TOF传感器变�?
 *---------------------------------------------------------------------------*/
uint16 tof[2] = {0};
// uint8 count_tof = 0;
uint8 count_flag = 0;
static uint8 uart_cmd_buf[50];
static uint8 uart_cmd_rx_data[16];
static uint8 uart_cmd_index = 0;

static void uart_command_print_params(void)
{
    printf("%.2f,%.2f,", kpa, kpb);
    printf("%.2f,%.2f,", kd, kd_imu);
    printf("%.2f,%.2f,", kp_motor, ki_motor);
    printf("%d,", normal_speed);
    printf("%.2f,%.2f,", s, distance_after_huandao);
    printf("%.2f,", g_angle_turn);
    printf("%.2f,%.2f,", A_, B_);
    printf("%.2f\r\n", C_);
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

    if (cmd[0] == 't' && cmd[1] == '\0')
    {
        uart_output_mode++;
        if (uart_output_mode >= 3)
        {
            uart_output_mode = 0;
        }
        printf("mode,%d\r\n", uart_output_mode);
        return 1;
    }

    if (cmd[0] == '\0' || cmd[1] == '\0')
    {
        return 0;
    }

    value = uart_cmd_to_float((const char *)(cmd + 1));

    switch (cmd[0])
    {
        case 'a': kpa = value; break;
        case 'b': kpb = value; break;
        case 'd': kd = value; break;
        case 'D': kd_imu = value; break;

        case 'p':
            kp_motor = value;
            motor_left.Kp_motor = kp_motor;
            motor_right.Kp_motor = kp_motor;
            break;

        case 'i':
            ki_motor = value;
            motor_left.Ki_motor = ki_motor;
            motor_right.Ki_motor = ki_motor;
            break;

        case 'n':
            if ((int16)value == 0)
                CarControl_RequestSoftStop();
            else
                normal_speed = (int16)value;
            break;
        case 's': s = value; break;
        case 'y': distance_after_huandao = value; break;
        case 'z': g_angle_turn = value; break;
        case 'A': A_ = value; break;
        case 'B': B_ = value; break;
        case 'C': C_ = value; break;
        default: applied = 0; break;
    }

    if (applied)
    {
        uart_command_print_params();
    }

    return applied;
}

/*---------------------------------------------------------------------------
 * 定时器中断回�?IMU数据更新)
 * 说明：使用四元数算法进行姿态解算，更新euler.yaw等欧拉角
 *---------------------------------------------------------------------------*/
void pit_callback(void) {
    flag_gyro_z = 1;
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
        if (uart_cmd_index == 0 && uart_cmd_rx_data[i] == 't')
        {
            uart_cmd_buf[0] = 't';
            uart_cmd_buf[1] = '\0';
            uart_command_apply((char *)uart_cmd_buf);
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
 * 蜂鸣器初始化
 *---------------------------------------------------------------------------*/
void beep_init(void) {
    gpio_init(IO_P67, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    BEEP = 1;
}

/*---------------------------------------------------------------------------
 * 蜂鸣器测�?0.5s间隔)
 *---------------------------------------------------------------------------*/
void beep_test(void) {
    BEEP = !BEEP;
    system_delay_ms(500);
}

/*---------------------------------------------------------------------------
 * 障碍物判�?暂留空，根据需要实�?
 *---------------------------------------------------------------------------*/
void block_judgement(void) {
    
}


