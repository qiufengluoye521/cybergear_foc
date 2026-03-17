/**************************************************************************
 * File     : pmsm_currentctrl.c
 * Copyright: HPMicro
 * Des      : 
 * Data     : 2024-11
 * Author   : 上海先楫半导体科技有限公司 
 * Log      : 
 ***************************************************************************/
#include "pmsm_currentctrl.h"

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc_buff[3][BOARD_BLDC_ADC_SEQ_DMA_SIZE_IN_4BYTES];



/**********************************************
 * *void motor_angle_align_loop(MOTOR_PARA* motor, MOTOR_CONTROL_Global* global, qei_CalObj* qei_CalHdl)
 * * preposition
 * *@Input: Motor structure, motor control global structure, qei structure
 * *#Output：None
 * ***************************************/
void motor_angle_align_loop(MOTOR_PARA* motor, MOTOR_CONTROL_Global* global, qei_CalObj* qei_CalHdl)
{
    motor->foc_para.samplcurpar.adc_u = ((adc_buff[0][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    motor->foc_para.samplcurpar.adc_v = ((adc_buff[1][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    motor->foc_para.samplcurpar.adc_w = ((adc_buff[2][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    motor->foc_para.electric_angle = HPM_MOTOR_MATH_FL_MDF(global->commu_theta);
    motor->foc_para.currentdpipar.target = HPM_MOTOR_MATH_FL_MDF(global->commu_IdRef);
    motor->foc_para.currentqpipar.target = HPM_MOTOR_MATH_FL_MDF(0);

    motor->foc_para.func_dqsvpwm(&motor->foc_para);
    motor->foc_para.pwmpar.pwmout.func_set_pwm(&motor->foc_para.pwmpar.pwmout);
}
void  motor_highspeed_loop(MOTOR_PARA* motor, MOTOR_CONTROL_Global* global, qei_CalObj* qei_CalHdl)
{
    motor->foc_para.samplcurpar.adc_u = ((adc_buff[0][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    motor->foc_para.samplcurpar.adc_v = ((adc_buff[1][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    motor->foc_para.samplcurpar.adc_w = ((adc_buff[2][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    electric_angle_cal(qei_CalHdl,global);
    //motor->foc_para.electric_angle = qei_CalHdl->elec_angle;   
    if (1 == global->motor_CW)
    { 
      motor->foc_para.func_dqsvpwm(&motor->foc_para);
      motor->foc_para.pwmpar.pwmout.func_set_pwm(&motor->foc_para.pwmpar.pwmout);
    }
    else
    {
       motor->foc_para.currentdpipar.target = 0;
       motor->foc_para.currentqpipar.target = 0;
       motor->foc_para.currentdpipar.mem = 0;
       motor->foc_para.currentqpipar.mem = 0;
       motor->foc_para.currentdpipar.cur = 0;
       motor->foc_para.currentqpipar.cur = 0;
       motor->foc_para.currentdpipar.outval = 0;
       motor->foc_para.currentqpipar.outval = 0;
       disable_all_pwm_output(BOARD_PMSM0PWM);
       //motor->foc_para.pwmpar.pwmout.func_set_pwm(&motor->foc_para.pwmpar.pwmout);
    }
    //motor->foc_para.speedcalpar.speedtheta = qei_CalHdl->pos;
    motor->foc_para.speedcalpar.func_getspd(&motor->foc_para.speedcalpar);
}

/**********************************************
 * *void isr_current_loop()
 * * 20K current ctrl ISR
 * *@Input: None
 * *#Output：None
 * ***************************************/
void isr_current_loop(void)
{
    uint32_t status, status1, status1cnt;
    float user_give_angle = 0;
    //status = pwm_get_status(BOARD_PMSM0PWM);
    status = pwmv2_get_cmp_irq_status(BOARD_PMSM0PWM);
     gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 1);
    if (PWM_IRQ_CMP(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP) == (status & PWM_IRQ_CMP(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP))) 
    { 
        pwmv2_clear_cmp_irq_status(BOARD_PMSM0PWM, status); 
        motor.adc_trig_event_callback(&motor,&Motor_Control_Global,&qeiCalObj); 
    }
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 0);
}

SDK_DECLARE_EXT_ISR_M(BOARD_PMSM0APP_PWM_IRQ, isr_current_loop)


