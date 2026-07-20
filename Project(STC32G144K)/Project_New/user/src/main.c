#include "headfile.h"
#include "element.h"
#include "spatial_features.h"
#include "seesaw.h"
#include "cylinder.h"
#include "wall.h"

volatile uint8 send_flag = 1;
uint8 uart_output_mode = 0;

void uart_telemetry_print(void)
{
    const spatial_features_t *features = SpatialFeatures_Get();

    switch (uart_output_mode)
    {
        case 0:
            printf("%d,", flag);
            printf("%d,%d,", motor_left.setspeed, motor_left.encoder_data);
            printf("%d,", motor_left.duty1);
            // printf("%d,%d,", motor_right.setspeed, motor_right.encoder_data);
            printf("%d,%.1f,", normal_speed, encoder_ave);
            printf("%.2f,", aaddcc.err_dir);
            printf("%d\r\n", voltage_battery_get_mv());
            break;

        case 1:
            printf("%d,", flag);
            printf("%d,%d,", ad_ave[0], ad_ave[1]);
            printf("%d,%d,", ad_ave[3], ad_ave[4]);
            printf("%.2f,", aaddcc.err_dir);
            printf("%.1f\r\n", encoder_ave);
            break;

        case 2:
            printf("%d,", flag);
            printf("%.2f,%.2f,", AD_ONE[0], AD_ONE[1]);
            printf("%.2f,%.2f,", AD_ONE[3], AD_ONE[4]);
            printf("%.2f,", aaddcc.err_dir);
            printf("%.1f,", encoder_ave);
            printf("%.2f\r\n",euler.yaw);
            break;

        case 3:
            // printf("%d,%d,%d,", imu660rc_gyro_x, imu660rc_gyro_y, imu660rc_gyro_z);
            // printf("%d,%d,%d,", imu660rc_acc_x, imu660rc_acc_y, imu660rc_acc_z);
            // printf("%.4f,%.4f,", q.q1, q.q2);
            printf("%.2f,%.2f,%.2f\r\n", euler.roll, euler.pitch, euler.yaw);
            // printf("%d,%.2f,", imu660rc_gyro_z, gyro_offset_z);
            // printf("%.3f,%.2f\r\n",
            //        (float)(imu660rc_gyro_z - gyro_offset_z) / imu660rc_transition_factor[1],
            //        euler.yaw);
            break;

        case 4:
            printf("%u,%u,", Element_GetRouteIndex(), Element_GetCurrent());
            printf("%u,%u,%u,", Seesaw_GetState(), Cylinder_GetState(),
                   Wall_GetState());
            printf("%.3f,%.3f,", features->ay_g, features->az_g);
            printf("%.2f\r\n", features->climb_angle_deg);
            break;

        case 5:
            printf("%u,%u,%u,%u,", ad_ave[0], ad_ave[1],
                   ad_ave[3], ad_ave[4]);
            printf("%.2f,%u,%u,", euler.pitch,
                   Cylinder_EntryIsDetected(), Cylinder_IsOnSurface());
            printf("%u,%u,%u\r\n", Cylinder_HasExited(), pwm_fan,
                   suction_fan_pwm_cylinder);
            break;
        default:
            uart_output_mode = 0;
            break;
    }
}

void main(void)
{
    static imu_sample_t imu_sample;

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

    // suction_fan_on(9999);

    Quaternion_Init();
    while (imu660rc_init(IMU660RC_QUARTERNION_DISABLE))
    {
        printf("\r\nIMU660RC init error.");
        system_delay_ms(300);
    }
    system_delay_ms(1000);
    Gyro_Calibration(400);
    Element_Init();

    // Config_Init();
    // Huandao_Init();
    // CarControl_Init();

    // motor_left_control(2000);
    // motor_right_control(2000);

    normal_speed = 0;

    pit_ms_init(TIM0_PIT, 5, TM0_IRQHandler);
    pit_ms_init(TIM4_PIT, 5, TM4_IRQHandler);

    while (1)
    {
        uart_command_poll();

        if (flag_gyro_z)
        {
            uint8 ticks;

            EA = 0;
            ticks = gyro_update_ticks;
            gyro_update_ticks = 0;
            flag_gyro_z = 0;
            EA = 1;

            if (ticks == 0)
            {
                ticks = 1;
            }
            IMU_Update_Dt(0.005f * ticks);
            IMU_SampleCopy(&imu_sample);
            Element_ImuUpdate(&imu_sample);
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
