/**************************************************************************
 * File     : pmsm_protect.c
 * Copyright: HPMicro
 * Des      : motor protect:bus, current, temperature
 * Data     : 2024-11
 * Author   : 上海先楫半导体科技有限公司 
 * Log      : 
 ***************************************************************************/
#include "pmsm_protect.h"
MOTOR_CONTROL_PROTECT Motor_Control_Protect;
/**********************************************
 * *void motor_protect_param_init(void)
 * * protect param init
 * *@Input: None
 * *#Output：None
 * ***************************************/
void motor_protect_param_init(MOTOR_PARA *motor_par)
{
    Motor_Control_Protect.maxcurrent = 3000; // sample phase current range: 0~4096
    Motor_Control_Protect.maxmin_speed = 10 * BOARD_PMSM0_SPI_FOC_PHASE_COUNT_PER_REV;// / motor_par->foc_para.motorpar.i_poles_n;
    Motor_Control_Protect.times_speed = 2;
    Motor_Control_Protect.times_current = 1;
    Motor_Control_Protect.times_encoder = 1;
    Motor_Control_Protect.times_temp = 5;
    Motor_Control_Protect.times_vol = 2;
    Motor_Control_Protect.maxvoltage = 34044; // 36V
    Motor_Control_Protect.minvoltage = 0;
    Motor_Control_Protect.maxtemp = 55893;//R=1.725K. temp=70//65535;
    Motor_Control_Protect.mintemp = 0;
}
/**********************************************
 * *void motor_protect_temp(uint16_t SAMPLE_UDC, MOTOR_CONTROL_PROTECT* ptr)
 * *ubus protect
 * *@Input: sample temp, temp protect parms
 * *#Output：None
 * ***************************************/
void motor_protect_temp(uint16_t SAMPLE_TEMP, MOTOR_CONTROL_PROTECT* ptr)
{
    static uint16_t m_times;
    m_times++;
    if ((m_times % ptr->times_temp) == 0)
    {
        if (SAMPLE_TEMP > ptr->maxtemp)
        {
            ptr->errorword |= 0x20;
        }
        else if (SAMPLE_TEMP < ptr->mintemp)
        {
            ptr->errorword |= 0x40;
        }
        else
        {
            //ptr->errorword |= 0x02;  //故障需排除后才可清0
        }
    }
    
}
/**********************************************
 * *void motor_protect_speed(MOTOR_PARA *par, MOTOR_CONTROL_PROTECT* ptr)
 * *speed and encoder protect
 * *@Input: target speed, feedback speed, feedback position
 * *#Output：None
 * ***************************************/
void motor_protect_speed(MOTOR_PARA *par, MOTOR_CONTROL_PROTECT* ptr, MOTOR_CONTROL_Global* ptr_global)
{
    static uint16_t m_times;
    m_times++;
    float m_speed_target, m_speed_cur;

    m_speed_target = par->speedloop_para.target;
    m_speed_cur = par->speedloop_para.cur;
    if ((m_times % ptr->times_speed) == 0)  //飞车保护
    {
        if (fabs(m_speed_target - m_speed_cur) > ptr->maxmin_speed)
        {
            ptr->errorword |= 0x08;
        }
        else
        {
            //ptr->errorword |= 0x02;  //故障需排除后才可清0
        }
    }
    if ((m_times % ptr->times_encoder) == 0)  //编码器断线，位置丢失保护
    {
        if ((par->foc_para.speedcalpar.speedtheta == 0) && 
        (par->speedloop_para.target != 0) &&
        (ptr_global->motor_CW == 1))
        {
            ptr->errorword |= 0x10;
        }
        else
        {
            //ptr->errorword |= 0x02;  //故障需排除后才可清0
        }
    }
}
/**********************************************
 * *void motor_protect_current(MOTOR_PARA *par, MOTOR_CONTROL_PROTECT* ptr)
 * *current protect
 * *@Input: feedback iq, current protect parms
 * *#Output：None
 * ***************************************/
void motor_protect_current(MOTOR_PARA *par, MOTOR_CONTROL_PROTECT* ptr)
{
    static uint16_t m_times;
    m_times++;
    if ((m_times % ptr->times_current) == 0)
    {
        if (par->foc_para.currentqpipar.cur > ptr->maxcurrent)
        {
            ptr->errorword |= 0x04;
        }
        else
        {
            //ptr->errorword |= 0x02;  //故障需排除后才可清0
        }
    }
    
}
/**********************************************
 * *void motor_protect_voltage(uint16_t SAMPLE_UDC, MOTOR_CONTROL_PROTECT* ptr)
 * *ubus protect
 * *@Input: sample udc, voltage protect parms
 * *#Output：None
 * ***************************************/
void motor_protect_voltage(uint16_t SAMPLE_UDC, MOTOR_CONTROL_PROTECT* ptr)
{
    static uint16_t m_times;
    m_times++;
    if ((m_times % ptr->times_vol) == 0)
    {
        ptr->maxvoltage = 34044; // 36V
        ptr->minvoltage = 0;
        if (SAMPLE_UDC > ptr->maxvoltage)
        {
            ptr->errorword |= 0x01;
        }
        else if (SAMPLE_UDC < ptr->minvoltage)
        {
            ptr->errorword |= 0x02;
        }
        else
        {
            //ptr->errorword |= 0x02;  //故障需排除后才可清0
        }
    }
    
}
/**********************************************
 * *void led_blink(uint16_t blink_time)
 * *protect led status
 * *@Input: 1ms flag
 * *#Output：None
 * ***************************************/
void led_blink(uint16_t blink_time)
{
    static uint16_t m_blink_time = 0;
    
    m_blink_time++;
    if (blink_time == 0)
    {
        gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOC, 18, 0);
        return;
    }
    if (((m_blink_time / blink_time) % 2) == 1)
    {    
        gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOC, 18, 1);
    }
    else
    {
        gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOC, 18, 0);
    }


}
/**********************************************
 * *void motor_operation_led_ctrl(MOTOR_CONTROL_PROTECT* ptr)
 * *Motor protect control led
 * *@Input: control word
 * *#Output：None
 * ***************************************/
void motor_operation_led_ctrl(MOTOR_CONTROL_PROTECT* ptr)
{
    if  ((ptr->errorword & 0x03) != 0)
    {
        led_blink(50);
    }
    else if((ptr->errorword & 0x04) != 0)
    {
        led_blink(500);
    }
    else if((ptr->errorword & 0x18) != 0)
    {
        led_blink(2000);
    }
    else if((ptr->errorword & 0x60) != 0)
    {
        led_blink(5000);
    }
    else
    {
        led_blink(0);
    }

}
/**********************************************
 * *void motor_protect(MOTOR_CONTROL_Global* global)
 * *Motor protect
 * *@Input: control word
 * *#Output：None
 * ***************************************/
void motor_protect(MOTOR_PARA *par, MOTOR_CONTROL_Global* global)
{

    uint16_t m_voltage_result;
    uint16_t m_temp_result;

    adc16_get_prd_result(BOARD_APP_ADC16_BASE, BOARD_APP_ADC16_UDC_CH, &m_voltage_result);
    adc16_get_prd_result(BOARD_APP_ADC16_BASE, BOARD_APP_ADC16_TEMP_CH, &m_temp_result);
    //printf("Period Mode - %s [channel %02d] - Result: 0x%04x\n", BOARD_APP_ADC16_NAME, BOARD_APP_ADC16_CH_1, result);
    motor_protect_voltage(m_voltage_result, &Motor_Control_Protect);
    motor_protect_current(par, &Motor_Control_Protect);
    motor_protect_speed(par, &Motor_Control_Protect, &global);
    motor_protect_temp(m_temp_result, &Motor_Control_Protect);
    motor_operation_led_ctrl(&Motor_Control_Protect);

}