/*
 * Copyright (c) 2021 hpmicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "board.h"
#include "pmsm_currentctrl_svpwm.h"
#include "hpm_pwmv2_drv.h"
#include "bldc_foc_cfg.h"
#include "pmsm_define.h"
#include "bldc_foc_callback.h"

const uint8_t pwm_uvw_conversion_tbl[4][6] ={
  {
    BOARD_BLDC_UH_PWM_OUTPIN,
    BOARD_BLDC_UL_PWM_OUTPIN,
    BOARD_BLDC_VH_PWM_OUTPIN,
    BOARD_BLDC_VL_PWM_OUTPIN,
    BOARD_BLDC_WH_PWM_OUTPIN,
    BOARD_BLDC_WL_PWM_OUTPIN 
  },
  {
    BOARD_BLDC_UH_PWM_OUTPIN,
    BOARD_BLDC_UL_PWM_OUTPIN,
    BOARD_BLDC_VH_PWM_OUTPIN,
    BOARD_BLDC_VL_PWM_OUTPIN,
    BOARD_BLDC_WH_PWM_OUTPIN,
    BOARD_BLDC_WL_PWM_OUTPIN 
  }
};

PWMV2_Type* motor_pwm_tbl[4] ={
  BOARD_STEPPER0PWM,
};

void bldc_pwm_enable(uint8_t motor_index,uint8_t pin_name)
{
  pwm_disable_pwm_sw_force_output(motor_pwm_tbl[motor_index],pwm_uvw_conversion_tbl[motor_index][pin_name]);
}

void bldc_pwm_disable(uint8_t motor_index,uint8_t pin_name)
{
  pwm_enable_pwm_sw_force_output(motor_pwm_tbl[motor_index],pwm_uvw_conversion_tbl[motor_index][pin_name]);
}
/*
void pwm_cmp_force_value_step(PWMV2_Type *pwm_x, uint8_t index, uint32_t cmp)
{

    pwm_x->CMP[index] = PWM_CMP_CMP_SET(cmp);

}
void pwm_update_raw_cmp_central_aligned_step(PWMV2_Type *pwm_x, uint8_t cmp1_index,
                                       uint8_t cmp2_index, uint32_t target_cmp1, uint32_t target_cmp2)
{
    uint32_t reload = PWM_RLD_RLD_GET(pwm_x->RLD);
    if (!target_cmp1) {
        target_cmp1 = reload + 1;
    }
    if (!target_cmp2) {
        target_cmp2 = reload + 1;
    }

    pwm_cmp_force_value_step(pwm_x, cmp1_index, target_cmp1);
    pwm_cmp_force_value_step(pwm_x, cmp2_index, target_cmp2);
   
}
*/


void bldc_foc_pwmset(BLDC_CONTROL_PWMOUT_PARA *par)
{
  uint32_t pwm_reload;
  uint32_t pwm_u_half, pwm_v_half, pwm_w_half;

  pwm_reload = par->i_pwm_reload >> 1;
  switch (par->i_motor_id){
    case BLDC_MOTOR0_INDEX:
        //par->pwm_u = 7000;
        //par->pwm_v = 7000;
        //par->pwm_w = 7000;
        pwm_u_half =  par->pwm_u >> 1;
        pwm_v_half =  par->pwm_v >> 1;
        pwm_w_half =  par->pwm_w >> 1;

       
        /*pwm_cmp_force_value(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_0, PWM_CMP_CMP_SET((pwm_reload + pwm_u_half)));
        pwm_cmp_force_value(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_1, PWM_CMP_CMP_SET((pwm_reload - pwm_u_half)));
        pwm_cmp_force_value(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_2, PWM_CMP_CMP_SET((pwm_reload + pwm_v_half)));
        pwm_cmp_force_value(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_3, PWM_CMP_CMP_SET((pwm_reload - pwm_v_half)));
        pwm_cmp_force_value(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_4, PWM_CMP_CMP_SET((pwm_reload + pwm_w_half)));
        pwm_cmp_force_value(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_5, PWM_CMP_CMP_SET((pwm_reload - pwm_w_half)));*/
        
        pwmv2_shadow_register_unlock(BOARD_PMSM0PWM);
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_0 + 1, (pwm_reload - pwm_u_half), 0, false);
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_1 + 1, (pwm_reload + pwm_u_half), 0, false);
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_2 + 1, (pwm_reload - pwm_v_half), 0, false);
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_3 + 1, (pwm_reload + pwm_v_half), 0, false);
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_4 + 1, (pwm_reload - pwm_w_half), 0, false);
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, BOARD_PMSM0PWM_CMP_INDEX_5 + 1, (pwm_reload + pwm_w_half), 0, false);
        pwmv2_shadow_register_lock(BOARD_PMSM0PWM);
        //pwm_issue_shadow_register_lock_event(BOARD_PMSM0PWM);
    break;

  default:
    break;
  }
}
void disable_all_pwm_output(PWMV2_Type *ptr)
{
 /*   if ((ptr == HPM_PWM0))
    {
        pwm_config_force_cmd_timing(BOARD_PMSM0PWM, pwm_force_immediately);
        pwm_enable_pwm_sw_force_output(BOARD_PMSM0PWM, BOARD_PMSM0_UH_PWM_OUTPIN);
        pwm_enable_pwm_sw_force_output(BOARD_PMSM0PWM, BOARD_PMSM0_UL_PWM_OUTPIN);
        pwm_enable_pwm_sw_force_output(BOARD_PMSM0PWM, BOARD_PMSM0_VH_PWM_OUTPIN);
        pwm_enable_pwm_sw_force_output(BOARD_PMSM0PWM, BOARD_PMSM0_VL_PWM_OUTPIN);
        pwm_enable_pwm_sw_force_output(BOARD_PMSM0PWM, BOARD_PMSM0_WH_PWM_OUTPIN);
        pwm_enable_pwm_sw_force_output(BOARD_PMSM0PWM, BOARD_PMSM0_WL_PWM_OUTPIN);
        pwm_set_force_output(BOARD_PMSM0PWM,
                            PWM_FORCE_OUTPUT(BOARD_PMSM0_UH_PWM_OUTPIN, pwm_output_0)
                            | PWM_FORCE_OUTPUT(BOARD_PMSM0_UL_PWM_OUTPIN, pwm_output_0)
                            | PWM_FORCE_OUTPUT(BOARD_PMSM0_VH_PWM_OUTPIN, pwm_output_0)
                            | PWM_FORCE_OUTPUT(BOARD_PMSM0_VL_PWM_OUTPIN, pwm_output_0)
                            | PWM_FORCE_OUTPUT(BOARD_PMSM0_WH_PWM_OUTPIN, pwm_output_0)
                            | PWM_FORCE_OUTPUT(BOARD_PMSM0_WL_PWM_OUTPIN, pwm_output_0));
        pwm_enable_sw_force(BOARD_PMSM0PWM);
    }
*/
    pwmv2_set_force_update_time(BOARD_PMSM0PWM, BOARD_BLDC_UH_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(BOARD_PMSM0PWM, BOARD_BLDC_UL_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(BOARD_PMSM0PWM, BOARD_BLDC_VH_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(BOARD_PMSM0PWM, BOARD_BLDC_VL_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(BOARD_PMSM0PWM, BOARD_BLDC_WH_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(BOARD_PMSM0PWM, BOARD_BLDC_WL_PWM_OUTPIN, pwm_force_immediately);

    pwmv2_force_update_time_by_shadow(BOARD_PMSM0PWM, BOARD_BLDC_UH_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(BOARD_PMSM0PWM, BOARD_BLDC_UL_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(BOARD_PMSM0PWM, BOARD_BLDC_VH_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(BOARD_PMSM0PWM, BOARD_BLDC_VL_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(BOARD_PMSM0PWM, BOARD_BLDC_WH_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(BOARD_PMSM0PWM, BOARD_BLDC_WL_PWM_OUTPIN, pwm_force_update_shadow_immediately);

    pwmv2_enable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwmv2_enable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwmv2_enable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwmv2_enable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwmv2_enable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwmv2_enable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_WL_PWM_OUTPIN);

    pwmv2_enable_software_force(BOARD_PMSM0PWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwmv2_enable_software_force(BOARD_PMSM0PWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwmv2_enable_software_force(BOARD_PMSM0PWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwmv2_enable_software_force(BOARD_PMSM0PWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwmv2_enable_software_force(BOARD_PMSM0PWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwmv2_enable_software_force(BOARD_PMSM0PWM, BOARD_BLDC_WL_PWM_OUTPIN);

    pwmv2_shadow_register_unlock(BOARD_PMSM0PWM);
    pwmv2_force_output(BOARD_PMSM0PWM, BOARD_BLDC_UH_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(BOARD_PMSM0PWM, BOARD_BLDC_UL_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(BOARD_PMSM0PWM, BOARD_BLDC_VH_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(BOARD_PMSM0PWM, BOARD_BLDC_VL_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(BOARD_PMSM0PWM, BOARD_BLDC_WH_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(BOARD_PMSM0PWM, BOARD_BLDC_WL_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_shadow_register_lock(BOARD_PMSM0PWM);
}
void enable_all_pwm_output(PWMV2_Type *ptr)
{
 /*   if ((ptr == HPM_PWM0))
    {
        pwm_disable_sw_force(BOARD_PMSM0PWM);
    }
*/
    pwmv2_disable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwmv2_disable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwmv2_disable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwmv2_disable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwmv2_disable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwmv2_disable_force_by_software(BOARD_PMSM0PWM, BOARD_BLDC_WL_PWM_OUTPIN);
}
