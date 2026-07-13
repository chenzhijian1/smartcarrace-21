#include "my_motor.h"
#include "config.h"
#include "car_control.h"

motor_struct motor_left;
motor_struct motor_right;

uint16 pwm_fan = 0;

static uint16 motor_pwm_abs(int pwm)
{
    if (pwm < 0)
    {
        pwm = -pwm;
    }

    if (pwm > MOTOR_PWM_MAX)
    {
        pwm = MOTOR_PWM_MAX;
    }

    return (uint16)pwm;
}

static void drv8701e_wake(gpio_pin_enum nsleep_pin)
{
    gpio_init(nsleep_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(1);
    gpio_set_level(nsleep_pin, GPIO_LOW);
    system_delay_us(20);
    gpio_set_level(nsleep_pin, GPIO_HIGH);
}

void encoder_init(void)
{
    encoder_dir_init(SPEEDL_ENCODER, SPEEDL_LSB_PIN, SPEEDL_DIR_PIN);
    encoder_dir_init(SPEEDR_ENCODER, SPEEDR_LSB_PIN, SPEEDR_DIR_PIN);
}

void encoder_get(void)
{
#if CAR_REVERSED_RUN
    int16 encoder_swap;
#endif

    motor_left.encoder_data = encoder_get_count(SPEEDL_ENCODER);
    motor_right.encoder_data = encoder_get_count(SPEEDR_ENCODER);

    encoder_clear_count(SPEEDL_ENCODER);
    encoder_clear_count(SPEEDR_ENCODER);

    if (SPEEDL_ENCODER_REVERSE)
    {
        motor_left.encoder_data = -motor_left.encoder_data;
    }
    if (SPEEDR_ENCODER_REVERSE)
    {
        motor_right.encoder_data = -motor_right.encoder_data;
    }

#if CAR_REVERSED_RUN
    // A 180-degree vehicle rotation maps logical left/right to physical
    // right/left, and reverses the forward direction of both encoders.
    encoder_swap = motor_left.encoder_data;
    motor_left.encoder_data = -motor_right.encoder_data;
    motor_right.encoder_data = -encoder_swap;
#endif
}

void encoder(void)
{
    encoder_ave += (0.017 * motor_left.encoder_data + 0.017 * motor_right.encoder_data) / 2;
}

void encoder_clear(void)
{
    encoder_ave = 0.0;
}

void motor_driver_init(void)
{
    drv8701e_wake(MOTOR_LEFT_NSLEEP_PIN);
    drv8701e_wake(MOTOR_RIGHT_NSLEEP_PIN);

    gpio_init(MOTOR_LEFT_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(MOTOR_LEFT_PWM, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_PWM_FREQ, 0);

    MOTOR_LEFT_DIR = 0;
    MOTOR_RIGHT_DIR = 0;

    motor_struct_parameter_init(&motor_left, 0);
    motor_struct_parameter_init(&motor_right, 0);
}

int motor_pwm_limit(int pwm_input)
{
    if (pwm_input > MOTOR_PWM_MAX)
    {
        return MOTOR_PWM_MAX;
    }
    if (pwm_input < -MOTOR_PWM_MAX)
    {
        return -MOTOR_PWM_MAX;
    }
    return pwm_input;
}

void motor_struct_parameter_init(motor_struct *sptr, int16 sspeed)
{
    sptr->duty1 = 0;
    sptr->setspeed = sspeed;
    sptr->actspeed = 0;
    sptr->encoder_data = 0;

    sptr->err = 0;
    sptr->err1 = 0;
    sptr->err2 = 0;

    sptr->out_p = 0;
    sptr->out_i = 0;
    sptr->out_d = 0;
    sptr->out_motor_pid = 0;

    sptr->Kp_motor = kp_motor;
    sptr->Ki_motor = ki_motor;
    sptr->Kd_motor = kd_motor;
}

void motor_left_set_direction(motor_dir_e dir)
{
    if (dir == MOTOR_FORWARD)
    {
        MOTOR_LEFT_DIR = 1;
    }
    else
    {
        MOTOR_LEFT_DIR = 0;
    }
}

void motor_left_brake(void)
{
    pwm_set_duty(MOTOR_LEFT_PWM, 0);
}

void motor_left_set_pwm(int pwm)
{
    pwm_set_duty(MOTOR_LEFT_PWM, motor_pwm_abs(pwm));
}

static void motor_left_hw_control(int pwm)
{
    pwm = motor_pwm_limit(pwm);

    if (pwm > 0)
    {
        motor_left_set_direction(MOTOR_FORWARD);
    }
    else if (pwm < 0)
    {
        motor_left_set_direction(MOTOR_REVERSE);
    }

    pwm_set_duty(MOTOR_LEFT_PWM, motor_pwm_abs(pwm));
}

void motor_right_set_direction(motor_dir_e dir)
{
    if (dir == MOTOR_FORWARD)
    {
        MOTOR_RIGHT_DIR = 1;
    }
    else
    {
        MOTOR_RIGHT_DIR = 0;
    }
}

void motor_right_brake(void)
{
    pwm_set_duty(MOTOR_RIGHT_PWM, 0);
}

void motor_right_set_pwm(int pwm)
{
    pwm_set_duty(MOTOR_RIGHT_PWM, motor_pwm_abs(pwm));
}

static void motor_right_hw_control(int pwm)
{
    pwm = motor_pwm_limit(pwm);

    if (pwm > 0)
    {
        motor_right_set_direction(MOTOR_FORWARD);
    }
    else if (pwm < 0)
    {
        motor_right_set_direction(MOTOR_REVERSE);
    }

    pwm_set_duty(MOTOR_RIGHT_PWM, motor_pwm_abs(pwm));
}

void motor_left_control(int pwm)
{
#if CAR_REVERSED_RUN
    motor_right_hw_control(pwm);
#else
    motor_left_hw_control(pwm);
#endif
}

void motor_right_control(int pwm)
{
#if CAR_REVERSED_RUN
    motor_left_hw_control(pwm);
#else
    motor_right_hw_control(pwm);
#endif
}

int16 motor_closed_loop_control(motor_struct *sptr)
{
    int tspeed;

    sptr->err = sptr->setspeed - sptr->encoder_data;

    tspeed = (int16)(sptr->Kp_motor * (sptr->err - sptr->err1) +
                     sptr->Ki_motor * sptr->err +
                     sptr->Kd_motor * (sptr->err - 2 * sptr->err1 + sptr->err2));

    sptr->err2 = sptr->err1;
    sptr->err1 = sptr->err;

    sptr->out_motor_pid = MINMAX(sptr->out_motor_pid + tspeed, -10000, 10000);

    return sptr->out_motor_pid;
}

void motor_control(int16 speed_l, int16 speed_r)
{
    // if (normal_speed == 0)
    // {
    //     motor_left.setspeed = 0;
    //     motor_right.setspeed = 0;
    // }
    // else
    // {
        motor_left.setspeed = speed_l;
        motor_right.setspeed = speed_r;
    // }

    motor_closed_loop_control(&motor_left);
    motor_closed_loop_control(&motor_right);

    // motor_left.duty1 = motor_left.setspeed < 1000 ?
    //                    motor_left.setspeed * 1000 / 65 + motor_left.out_motor_pid :
    //                    1000 + (motor_left.setspeed - 65) / 40 * 500 + motor_left.out_motor_pid;
    // motor_right.duty1 = motor_right.setspeed < 1000 ?
    //                     motor_right.setspeed * 1000 / 45 + motor_right.out_motor_pid :
    //                     1000 + (motor_right.setspeed - 45) / 40 * 500 + motor_right.out_motor_pid;
    motor_left.duty1 = motor_left.out_motor_pid;
    motor_right.duty1 = motor_right.out_motor_pid;

    motor_left_control(motor_left.duty1);
    motor_right_control(motor_right.duty1);
}

static void motor_closed_loop_reset(motor_struct *sptr)
{
    sptr->setspeed = 0;
    sptr->duty1 = 0;
    sptr->err = 0;
    sptr->err1 = 0;
    sptr->err2 = 0;
    sptr->out_p = 0;
    sptr->out_i = 0;
    sptr->out_d = 0;
    sptr->out_motor_pid = 0;
}

void motor_control_stop(void)
{
    motor_closed_loop_reset(&motor_left);
    motor_closed_loop_reset(&motor_right);
    motor_left_brake();
    motor_right_brake();
}

void suction_fan_init(void)
{
    drv8701e_wake(SUCTION_FAN_NSLEEP_PIN);

    gpio_init(SUCTION_FAN_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(SUCTION_FAN_PWM, SUCTION_FAN_PWM_FREQ, 0);
    SUCTION_FAN_DIR = 0;
}

void suction_fan_set_direction(motor_dir_e dir)
{
    if (dir == MOTOR_FORWARD)
    {
        SUCTION_FAN_DIR = 1;
    }
    else
    {
        SUCTION_FAN_DIR = 0;
    }
}

void suction_fan_brake(void)
{
    pwm_set_duty(SUCTION_FAN_PWM, 0);
}

void suction_fan_set_pwm(int pwm)
{
    pwm_set_duty(SUCTION_FAN_PWM, motor_pwm_abs(pwm));
}

void suction_fan_control(int pwm)
{
    pwm = motor_pwm_limit(pwm);

    suction_fan_set_direction(MOTOR_FORWARD);

    pwm_set_duty(SUCTION_FAN_PWM, motor_pwm_abs(pwm));
}

void suction_fan_on(int pwm)
{
    suction_fan_control(pwm);
}

void suction_fan_off(void)
{
    pwm_fan = 0;
    SUCTION_FAN_DIR = 1;
    suction_fan_brake();
}
