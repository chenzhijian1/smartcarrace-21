#include "headfile.h"

uint8 send_flag = 1;

int8 duty = 0;
int8 dir = 1;

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    interrupt_global_enable();

    gpio_init(IO_P52, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    direction_adc_init();

    encoder_init();
    motor_driver_init();
    suction_fan_init();

    // ips114_init();
    // ips114_clear(RGB565_BLACK);

    if(wireless_uart_init())                                          // 判断初始化是否成功
    {
        while(1)                                                      // 初始化失败后进入死循环
        {
            gpio_toggle_level(IO_P52);                                  // 翻转 LED 引脚输出电平 控制 LED 亮灭
            system_delay_ms(100);                                     // 短延时快速闪灯表示异常
        }
    }
    
    Quaternion_Init();
    while (imu963ra_init())
    {
        printf("\r\nIMU963RA init error.");
        system_delay_ms(300);
    }
    Gyro_Calibration(200);

    // Config_Init();
    // Huandao_Init();
    // CarControl_Init();

    // motor_left_control(2000);
    // motor_right_control(2000);

    // suction_fan_control(6000);

    normal_speed = 0;

    pit_ms_init(TIM0_PIT, 5, TM0_IRQHandler);
    pit_ms_init(TIM4_PIT, 5, TM4_IRQHandler);

    while (1)
    {
        gpio_set_level(IO_P07, GPIO_HIGH);

        uart_command_poll();

        if (flag_gyro_z)
        {
            flag_gyro_z = 0;
            IMU_Update();
        }

        if (flag_adc)
        {
            flag_adc = 0;
            direction_adc_get();
        }
        // UI loop
        // UI_KeyScan();
        // UI_Display();

        if (send_flag)
        {
            send_flag = 0;

            // printf("%d,%d\r\n", motor_left.encoder_data, motor_right.encoder_data);

            // printf("%d,%d,%d,%d,%d,%d,%.2f\r\n",
            //        imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z,
            //        imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z, euler.yaw);

            // printf("%.2f\r\n", euler.yaw);

            printf("%d,%d,%d,%d,%d,%.2f,%d\r\n",
                   motor_left.setspeed, motor_left.encoder_data,
                   motor_right.setspeed, motor_right.encoder_data,
                   normal_speed,
                   aaddcc.err_dir,
                   voltage_battery_get_mv());

            // printf("%d,%d,%d,%d,%.2f\r\n", ad_ave[0], ad_ave[1], ad_ave[3], ad_ave[4], aaddcc.err_dir);

        }

        // system_delay_ms(20);
    }
}
