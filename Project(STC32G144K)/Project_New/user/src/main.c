#include "headfile.h"

uint8 send_flag = 1;
uint8 uart_output_mode = 0;

void uart_telemetry_print(void)
{
    switch (uart_output_mode)
    {
        case 0:
            printf("%d,%d,", motor_left.setspeed, motor_left.encoder_data);
            printf("%d,%d,", motor_right.setspeed, motor_right.encoder_data);
            printf("%d,", normal_speed);
            printf("%.2f,", aaddcc.err_dir);
            printf("%d\r\n", voltage_battery_get_mv());
            break;

        case 1:
            printf("%d,%d,", ad_ave[0], ad_ave[1]);
            printf("%d,%d,", ad_ave[3], ad_ave[4]);
            printf("%.2f\r\n", aaddcc.err_dir);
            break;

        case 2:
            printf("%.2f,%.2f,", AD_ONE[0], AD_ONE[1]);
            printf("%.2f,%.2f,", AD_ONE[3], AD_ONE[4]);
            printf("%.2f\r\n", aaddcc.err_dir);
            break;

        default:
            uart_output_mode = 0;
            break;
    }
}

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

    if(wireless_uart_init()) {                                         // 判断初始化是否成�?
        while(1) {                                                      // 初始化失败后进入死循�? 
            gpio_toggle_level(IO_P52);                                  // 翻转 LED 引脚输出电平 控制 LED 亮灭
            system_delay_ms(100);                                     // 短延时快速闪灯表示异�? 
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
            uart_telemetry_print();
        }

        // system_delay_ms(20);
    }
}
