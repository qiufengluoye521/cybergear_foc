
#include "pmsm_currentctrl.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_adc.h"
#include "hpm_gpio_drv.h"
#include "pmsm_define.h"
//#include "hpm_qei_drv.h"

#include "hpm_qeiv2_drv.h"
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc_buff[3][BOARD_BLDC_ADC_SEQ_DMA_SIZE_IN_4BYTES];

/**
 * @brief           calculate electrical Angle from by encoder position .
 */
void electric_angle_cal(qei_CalObj* qei_CalHdl,MOTOR_CONTROL_Global* global)
{
     qei_CalHdl->z =qeiv2_get_current_count(BOARD_PMSM0_QEI_BASE, qeiv2_counter_type_z)&0x1fffff;
     qei_CalHdl->ph = qeiv2_get_current_count(BOARD_PMSM0_QEI_BASE, qeiv2_counter_type_phase)&0x1fffff;
     if(qei_CalHdl->z >= (0x200000 >> 1))
     {
        qei_CalHdl->pos = -(((qei_CalHdl->z - 0x200000)*qei_CalHdl->maxph)+qei_CalHdl->ph);
     }
     else
     {
        qei_CalHdl->pos =  -((qei_CalHdl->z*qei_CalHdl->maxph)+qei_CalHdl->ph);
     }
     uint32_t enc_cnt = qei_CalHdl->maxph/qei_CalHdl->motor_pole;
 
     qei_CalHdl->elec_angle = 360- (qei_CalHdl->ph%enc_cnt)*360.0/enc_cnt;

}


void motor_angle_align_loop(MOTOR_PARA* motor,MOTOR_CONTROL_Global* global,qei_CalObj* qei_CalHdl)
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
void  motor_highspeed_loop(MOTOR_PARA* motor,MOTOR_CONTROL_Global* global,qei_CalObj* qei_CalHdl)
{

    motor->foc_para.samplcurpar.adc_u = ((adc_buff[0][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    motor->foc_para.samplcurpar.adc_v = ((adc_buff[1][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    motor->foc_para.samplcurpar.adc_w = ((adc_buff[2][BOARD_PMSM0_ADC_TRG*4]&0xffff)>>4)&0xfff;
    electric_angle_cal(qei_CalHdl,global);
    motor->foc_para.electric_angle = qei_CalHdl->elec_angle;
   
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
 
    motor->foc_para.speedcalpar.speedtheta = qei_CalHdl->pos;

    motor->foc_para.speedcalpar.func_getspd(&motor->foc_para.speedcalpar);
   
}


