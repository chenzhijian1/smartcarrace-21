#include "headfile.h"
#include "isr.h"
#include "element.h"

#define LED P52

extern volatile uint8 send_flag;
uint8 cnt_send = 0;

static uint8 adc_tick = 0;

void INT0_IRQHandler(void) interrupt INT0_VECTOR
{
    LED = 0;
}

void INT1_IRQHandler(void) interrupt INT1_VECTOR
{
}

void INT2_IRQHandler(void) interrupt INT2_VECTOR
{
    INT2_CLEAR_FLAG;
}

void INT3_IRQHandler(void) interrupt INT3_VECTOR
{
    INT3_CLEAR_FLAG;
}

void INT4_IRQHandler(void) interrupt INT4_VECTOR
{
    INT4_CLEAR_FLAG;
}

void TM0_IRQHandler(void) interrupt TMR0_VECTOR
{
    TIM0_CLEAR_FLAG;
    
    pit_callback();

    if (++adc_tick >= 2)
    {
        adc_tick = 0;
        flag_adc = 1;
    }
}

void TM1_IRQHandler(void) interrupt TMR1_VECTOR
{
    TIM1_CLEAR_FLAG;
    
    
}

void TM2_IRQHandler(void) interrupt TMR2_VECTOR
{
    TIM2_CLEAR_FLAG;

    
}

void TM3_IRQHandler(void) interrupt TMR3_VECTOR
{
    TIM3_CLEAR_FLAG;
}

void TM4_IRQHandler(void) interrupt TMR4_VECTOR
{
    TIM4_CLEAR_FLAG;

    if (uart_feedback_hold_ticks > 0)
    {
        uart_feedback_hold_ticks--;
        cnt_send = 0;
        send_flag = 0;
    }
    else if (++cnt_send >= 5)
    {
        cnt_send = 0;
        send_flag = 1;
    }
    
    encoder_get();
    encoder();

    CarControl_Update();

    if (car_stop_judge() || voltage_battery_is_low() ||
        (normal_speed == 0 && flag != CAR_STATE_SOFT_STOP))
    {
        suction_fan_off();
        motor_control_stop();
    }
    else if (Element_IsBrakeRequested())
    {
        motor_control_stop();
    }
    else if (flag == CAR_STATE_LAUNCH)
    {
        motor_control_stop();
    }
    else
    {
        motor_control(set_leftspeed, set_rightspeed);
    }
}

void UART1_IRQHandler(void) interrupt UART1_VECTOR
{
    if (UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }

    if (UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
    }
}

void UART2_IRQHandler(void) interrupt UART2_VECTOR
{
    if (UART2_GET_TX_FLAG)
    {
        UART2_CLEAR_TX_FLAG;
        busy[2] = 0;
    }

    if (UART2_GET_RX_FLAG)
    {
        UART2_CLEAR_RX_FLAG;
    }
}

void UART3_IRQHandler(void) interrupt UART3_VECTOR
{
    if (UART3_GET_TX_FLAG)
    {
        UART3_CLEAR_TX_FLAG;
        busy[3] = 0;
    }

    if (UART3_GET_RX_FLAG)
    {
        UART3_CLEAR_RX_FLAG;
    }
}

void DMA_UART3_IRQHandler(void) interrupt DMA_UR3R_VECTOR
{
    if (DMA_UR3R_STA & 0x01)
    {
        DMA_UR3R_STA &= ~0x01;
        uart_rx_start_buff(UART_3);

        if (uart_rx_handlers[UART_3] != 0)
        {
            uart_rx_handlers[UART_3](uart_rx_buff[UART_3][0]);
        }
    }

    if (DMA_UR3R_STA & 0x02)
    {
        DMA_UR3R_STA &= ~0x02;
        uart_rx_start_buff(UART_3);
    }
}

void UART4_IRQHandler(void) interrupt UART4_VECTOR
{
    if (UART4_GET_TX_FLAG)
    {
        UART4_CLEAR_TX_FLAG;
        busy[4] = 0;
    }

    if (UART4_GET_RX_FLAG)
    {
        UART4_CLEAR_RX_FLAG;
    }
}
