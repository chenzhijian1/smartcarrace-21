#include "my_peripheral.h"
#include "car_control.h"
#include "element.h"

/*============================================================================
 * 妯″潡璇存槑锛氬?栬?炬帶鍒舵ā鍧?
 * 鍔熻兘锛氳渹楦ｅ櫒銆侀檧铻轰华澶勭悊銆乀OF浼犳劅鍣?
 * 娉ㄦ剰锛氶檧铻轰华濮挎�佽В绠楀凡绉昏嚦quaternion.c妯″潡
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 闄�铻轰华鐩稿叧鍙橀噺
 *---------------------------------------------------------------------------*/
volatile uint8 flag_gyro_z = 0;
volatile uint8 gyro_update_ticks = 0;

/*---------------------------------------------------------------------------
 * TOF浼犳劅鍣ㄥ彉閲?
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

static void uart_command_print_result(char command)
{
    if (debug_mode)
    {
        switch (command)
        {
            case 'a': printf("kpa=%.2f\r\n", kpa); break;
            case 'b': printf("kpb=%.2f\r\n", kpb); break;
            case 'd': printf("kd=%.2f\r\n", kd); break;
            case 'D': printf("kd_imu=%.2f\r\n", kd_imu); break;
            case 'p': printf("kp_motor=%.2f\r\n", kp_motor); break;
            case 'i': printf("ki_motor=%.2f\r\n", ki_motor); break;
            case 'n': printf("normal_speed=%d\r\n", normal_speed); break;
            case 'l': printf("element_lap_target=%u\r\n", element_lap_target); break;
            case 'A': printf("A=%.3f\r\n", A_); break;
            case 'B': printf("B=%.3f\r\n", B_); break;
            case 'C': printf("C=%.3f\r\n", C_); break;
            default: break;
        }
    }
    else
    {
        printf("%.2f,%.2f,", kpa, kpb);
        printf("%.2f,%.2f,", kd, kd_imu);
        printf("%.2f,%.2f,", kp_motor, ki_motor);
        printf("%d,", normal_speed);
        printf("%.2f,%.2f,", A_, B_);
        printf("%.2f\r\n", C_);
    }
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

    // t0~t5: 选择串口输出模式，对应 main.c 中 uart_telemetry_print 的 case 0~5
    if (cmd[0] == 't' && cmd[1] >= '0' && cmd[1] <= '5' && cmd[2] == '\0')
    {
        uart_output_mode = (uint8)(cmd[1] - '0');
        if (!debug_mode) printf("mode,%d\r\n", uart_output_mode);
        uart_feedback_hold_start();
        return 1;
    }

    // f0/f4/f5: generic vehicle states; roundabout stages are internal to huandao.c
    if (cmd[0] == 'f' && cmd[1] >= '0' && cmd[1] <= '9' && cmd[2] == '\0')
    {
        uint8 requested_state = (uint8)(cmd[1] - '0');

        if (requested_state != CAR_STATE_NORMAL &&
            requested_state != CAR_STATE_LAUNCH &&
            requested_state != CAR_STATE_SOFT_STOP)
        {
            if (!debug_mode) printf("error,flag,%d\r\n", requested_state);
            uart_feedback_hold_start();
            return 1;
        }

        flag = requested_state;
        if (!debug_mode) printf("flag,%d\r\n", flag);
        uart_feedback_hold_start();
        return 1;
    }

    if (cmd[0] == 'h' &&
        cmd[1] >= '0' && cmd[1] < ('0' + HUANDAO_MAX_COUNT) &&
        cmd[2] != '\0')
    {
        uint8 index = (uint8)(cmd[1] - '0');
        value = uart_cmd_to_float((const char *)(cmd + 2));
        if (value < 0.0f)
            return 0;
        distance_before_huandao[index] = value;
        if (debug_mode) printf("distance_before_huandao[%u]=%.1f\r\n", index, distance_before_huandao[index]);
        else printf("huandao_distance,%u,%.1f\r\n", index, value);
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
        case 'a': kpa = value; break;       // a+数值: 修改方向环比例系数 kpa
        case 'b': kpb = value; break;       // b+数值: 修改方向环非线性系数 kpb
        case 'd': kd = value; break;        // d+数值: 修改方向环微分系数 kd
        case 'D': kd_imu = value; break;    // D+数值: 修改陀螺仪角速度反馈系数 kd_imu

        case 'p':                           // p+数值: 修改速度环 P，并同步到左右电机
            kp_motor = value;
            motor_left.Kp_motor = kp_motor;
            motor_right.Kp_motor = kp_motor;
            break;

        case 'i':                           // i+数值: 修改速度环 I，并同步到左右电机
            ki_motor = value;
            motor_left.Ki_motor = ki_motor;
            motor_right.Ki_motor = ki_motor;
            break;

        case 'n':                           // n+数值: 修改目标速度 normal_speed，n0 触发软停车
            if ((int16)value == 0)
                CarControl_RequestSoftStop();
            else
                Config_SetNormalSpeed((int16)value);
            break;

        case 'u':                           // u+数值: 修改负压风扇启动目标 PWM，占空比范围 0~10000
            suction_fan_pwm_start = (uint16)motor_pwm_limit((int)value);
            if (debug_mode) printf("suction_fan_pwm_start=%u\r\n", suction_fan_pwm_start);
            else printf("fan_start,%d\r\n", suction_fan_pwm_start);
            uart_feedback_hold_start();
            return 1;

        case 'r':
            if ((value != 0.0f && value != 1.0f) || normal_speed != 0)
            {
                applied = 0;
                break;
            }
            element_reverse_run = (uint8)value;
            Element_Init();
            if (debug_mode)
                printf("element_reverse_run=%u\r\n", element_reverse_run);
            else
                printf("element_reverse_run,%u\r\n", element_reverse_run);
            uart_feedback_hold_start();
            return 1;

        case 'l':
            if (value < 1.0f || value > 255.0f) applied = 0;
            else element_lap_target = (uint8)value;
            break;
        case 's': s = value; break;         // s+数值: 修改弯道速度衰减系数 s
        case 'A': A_ = value; break;        // A+数值: 修改电感误差公式系数 A_
        case 'B': B_ = value; break;        // B+数值: 修改电感误差公式系数 B_
        case 'C': C_ = value; break;        // C+数值: 修改电感误差公式系数 C_
        default: applied = 0; break;        // 未识别命令: 不修改参数，也不回显参数表
    }

    if (applied)
    {
        uart_command_print_result(cmd[0]);
    }

    return applied;
}

/*---------------------------------------------------------------------------
 * 瀹氭椂鍣ㄤ腑鏂?鍥炶??IMU鏁版嵁鏇存柊)
 * 璇存槑锛氫娇鐢ㄥ洓鍏冩暟绠楁硶杩涜?屽Э鎬佽В绠楋紝鏇存柊euler.yaw绛夋?ф媺瑙?
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
 * 铚傞福鍣ㄥ垵濮嬪寲
 *---------------------------------------------------------------------------*/
void beep_init(void) {
    gpio_init(IO_P67, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    BEEP = 1;
}

/*---------------------------------------------------------------------------
 * 铚傞福鍣ㄦ祴璇?0.5s闂撮殧)
 *---------------------------------------------------------------------------*/
void beep_test(void) {
    BEEP = !BEEP;
    system_delay_ms(500);
}

/*---------------------------------------------------------------------------
 * 闅滅?嶇墿鍒ゆ??鏆傜暀绌猴紝鏍规嵁闇�瑕佸疄鐜?
 *---------------------------------------------------------------------------*/
void block_judgement(void) {
    
}


