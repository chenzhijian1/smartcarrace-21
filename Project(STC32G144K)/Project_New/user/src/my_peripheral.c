#include "my_peripheral.h"
#include "car_control.h"
#include "element.h"

/*============================================================================
 * 模块说明：外设控制模�?
 * 功能：蜂鸣器、陀螺仪处理、TOF传感�?
 * 注意：陀螺仪姿态解算已移至quaternion.c模块
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 陀螺仪相关变量
 *---------------------------------------------------------------------------*/
volatile uint8 flag_gyro_z = 0;
volatile uint8 gyro_update_ticks = 0;

/*---------------------------------------------------------------------------
 * TOF传感器变�?
 *---------------------------------------------------------------------------*/
uint16 tof[2] = {0};
// uint8 count_tof = 0;
uint8 count_flag = 0;
static uint8 uart_cmd_buf[50];
static uint8 uart_cmd_rx_data[16];
static uint8 uart_cmd_index = 0;
volatile uint16 uart_feedback_hold_ticks = 0;
extern volatile uint8 send_flag;

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

    // t0~t5: ѡ�񴮿����ģʽ����Ӧ main.c �� uart_telemetry_print �� case 0~5
    if (cmd[0] == 't' && cmd[1] >= '0' && cmd[1] <= '9' && cmd[2] == '\0')
    {
        uart_output_mode = (uint8)(cmd[1] - '0');
        printf("mode,%d\r\n", uart_output_mode);
        uart_feedback_hold_start();
        return 1;
    }

    // f0~f9: ֱ���޸ĳ���״̬ flag������ f4 ������״̬��f5 ��������ͣ��״̬
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
        case 'a': kpa = value; break;       // a+��ֵ: �޸ķ��򻷱���ϵ�� kpa
        case 'b': kpb = value; break;       // b+��ֵ: �޸ķ��򻷷�����ϵ�� kpb
        case 'd': kd = value; break;        // d+��ֵ: �޸ķ���΢��ϵ�� kd
        case 'D': kd_imu = value; break;    // D+��ֵ: �޸������ǽ��ٶȷ���ϵ�� kd_imu

        case 'p':                           // p+��ֵ: �޸��ٶȻ� P����ͬ�������ҵ��
            kp_motor = value;
            motor_left.Kp_motor = kp_motor;
            motor_right.Kp_motor = kp_motor;
            break;

        case 'i':                           // i+��ֵ: �޸��ٶȻ� I����ͬ�������ҵ��
            ki_motor = value;
            motor_left.Ki_motor = ki_motor;
            motor_right.Ki_motor = ki_motor;
            break;

        case 'n':                           // n+��ֵ: �޸�Ŀ���ٶ� normal_speed��n0 ������ͣ��
            if ((int16)value == 0)
                CarControl_RequestSoftStop();
            else
                normal_speed = (int16)value;
            break;

        case 'u':                           // u+��ֵ: �޸ĸ�ѹ��������Ŀ�� PWM��ռ�ձȷ�Χ 0~10000
            suction_fan_pwm_start = (uint16)motor_pwm_limit((int)value);
            printf("fan_start,%d\r\n", suction_fan_pwm_start);
            uart_feedback_hold_start();
            return 1;

        case 'v':
            suction_fan_pwm_cylinder = (uint16)motor_pwm_limit((int)value);
            printf("fan_cylinder,%d\r\n", suction_fan_pwm_cylinder);
            uart_feedback_hold_start();
            return 1;

        case 's': s = value; break;         // s+��ֵ: �޸�����ٶ�˥��ϵ�� s
        case 'A': A_ = value; break;        // A+��ֵ: �޸ĵ����ʽϵ�� A_
        case 'B': B_ = value; break;        // B+��ֵ: �޸ĵ����ʽϵ�� B_
        case 'C': C_ = value; break;        // C+��ֵ: �޸ĵ����ʽϵ�� C_
        default: applied = 0; break;        // δʶ������: ���޸Ĳ�����Ҳ�����Բ�����
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
        if (uart_cmd_index == 1 && uart_cmd_buf[0] == 't' && uart_cmd_rx_data[i] >= '0' && uart_cmd_rx_data[i] <= '5')
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


