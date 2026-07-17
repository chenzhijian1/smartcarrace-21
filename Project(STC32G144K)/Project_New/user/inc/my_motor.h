#ifndef __MY_MOTOR_H
#define __MY_MOTOR_H

#include "headfile.h"

#define MINN(a, b) (((a) < (b)) ? (a) : (b))
#define MAXX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MINN(MAXX(input, low), upper)

// Encoder pins match README mapping.
#define SPEEDL_ENCODER          PWMB_ENCODER
#define SPEEDL_LSB_PIN          PWMB_ENCODER_CH1_P74
#define SPEEDL_DIR_PIN          PWMB_ENCODER_CH2_P75
#define SPEEDR_ENCODER          PWME_ENCODER
#define SPEEDR_LSB_PIN          PWME_ENCODER_CH1P_P70
#define SPEEDR_DIR_PIN          PWME_ENCODER_CH2P_P72
#define SPEEDL_ENCODER_REVERSE  0
#define SPEEDR_ENCODER_REVERSE  1

// The vehicle is installed 180 degrees from the original running direction.
// Keep the control layer in its original logical left/right coordinate system.
#define CAR_REVERSED_RUN        1

// DRV8701E groups follow E02_02_drv8701e_double_motor_contro_demo.
// 往后依次移动是新板子
#define MOTOR_LEFT_DIR_PIN      IO_P06
#define MOTOR_LEFT_DIR          P06
#define MOTOR_LEFT_PWM          PWMD_CH4_P53
#define MOTOR_LEFT_NSLEEP_PIN   IO_P36
#define MOTOR_LEFT_NSLEEP       P36

#define MOTOR_RIGHT_DIR_PIN     IO_P47
#define MOTOR_RIGHT_DIR         P47
#define MOTOR_RIGHT_PWM         PWMA_CH2P_P62
#define MOTOR_RIGHT_NSLEEP_PIN  IO_P36
#define MOTOR_RIGHT_NSLEEP      P36

#define SUCTION_FAN_DIR_PIN     IO_P07
#define SUCTION_FAN_DIR         P07
#define SUCTION_FAN_PWM         PWMA_CH1P_P60
#define SUCTION_FAN_NSLEEP_PIN  IO_P36
#define SUCTION_FAN_NSLEEP      P36

#define MOTOR_PWM_FREQ          17000
#define SUCTION_FAN_PWM_FREQ    33000
#define MOTOR_PWM_MAX           10000
#define MOTOR_PWM_MIN           10

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE,
} motor_dir_e;

typedef struct {
    int16 setspeed;
    int16 actspeed;
    int16 err;
    int16 err1;
    int16 err2;
    int16 encoder_data;
    int16 duty1;
    int16 out_p;
    int16 out_i;
    int16 out_d;
    float Kp_motor;
    float Ki_motor;
    float Kd_motor;
    float out_motor_pid;
} motor_struct;

extern motor_struct motor_left;
extern motor_struct motor_right;
extern uint16 pwm_fan;

void encoder_init(void);
void encoder_get(void);
void encoder(void);
void encoder_clear(void);

void motor_driver_init(void);
void motor_struct_parameter_init(motor_struct *sptr, int16 sspeed);
int motor_pwm_limit(int pwm_input);

void motor_left_set_direction(motor_dir_e dir);
void motor_left_set_pwm(int pwm);
void motor_left_control(int pwm);
void motor_left_brake(void);

void motor_right_set_direction(motor_dir_e dir);
void motor_right_set_pwm(int pwm);
void motor_right_control(int pwm);
void motor_right_brake(void);

int16 motor_closed_loop_control(motor_struct *sptr);
void motor_control(int16 speed_l, int16 speed_r);
void motor_control_stop(void);

void suction_fan_init(void);
void suction_fan_on(int pwm);
void suction_fan_off(void);

#endif
