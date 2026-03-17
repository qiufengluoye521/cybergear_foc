
#include "pmsm_init.h"
#include "pmsm_speedctrl.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_gptmr_drv.h"
#include "pmsm_currentctrl.h"
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "hpm_adc.h"
#include "hpm_synt_drv.h"
#include "pmsm_define.h"
#include "bldc_foc_cfg.h"
#include "pmsm_currentctrl_svpwm.h"

//#include "hpm_qei_drv.h"
#include "parm_global.h"

#include "hpm_qeiv2_drv.h"
#include "bldc_foc_callback.h"
#include "hpm_gpio_drv.h"



/**
 * @brief           struct pt init .
 */
void param_init(MOTOR_CONTROL_Global* global,qei_CalObj* qeiCalHdl,MOTOR_PARA* motor)
{
    global->motor_status = 0xff10;
    //global->zero_CW = 0x1210;
    global->flag_1ms = 1;
    qeiCalHdl->maxph = BOARD_PMSM0_QEI_FOC_PHASE_COUNT_PER_REV;
    qeiCalHdl->motor_pole = motor->foc_para.motorpar.i_poles_n;
}


/**
 * @brief           adc module init .
 */
void adc_module_cfg(adc_type* adc_typ,uint8_t adc_module,ADC16_Type* HPM_ADC_BASE)
{
    adc_typ->adc_base.adc16 = HPM_ADC_BASE;
    adc_typ->module = adc_module;
}


/**
 * @brief           init kp,ki,pi control function in speed/current loop.
 * @param[in,out]   par   motor foc para to operate on.
 * @param[in]       KP    proportional gain in speed/current loop.
 * @param[in]       KI    integral gain in speed/current loop.
 * @param[in]       MAX   output max,  min = -max .
 */
void pi_param_init(BLDC_CONTRL_PID_PARA *par, float KP, float KI, float MAX)
{
    par->func_pid  = hpm_mcl_bldc_foc_pi_contrl;
    par->i_kp      = HPM_MOTOR_MATH_FL_MDF(KP);
    par->i_ki      = HPM_MOTOR_MATH_FL_MDF(KI);
    par->i_max     = HPM_MOTOR_MATH_FL_MDF(MAX);

}


/**
 * @brief           init foc control para.
 * @param[in,out]   motor_par   foc contorl para to operate on.
 * @param[in]       MOTOR_ID    motor id.
 */
void motor_param_int(MOTOR_PARA *motor_par, uint8_t MOTOR_ID)
{

    //motor_par->foc_para.motorpar.func_smc_const = &hpm_mcl_smc_const_cal;
    motor_par->foc_para.motorpar.i_lstator_h = 0.003;
    motor_par->foc_para.motorpar.i_maxspeed_rs = 50;
    motor_par->foc_para.motorpar.i_phasecur_a = 3;//paraObj[MOTOR_ID-1].MotorObj.MotorParaObj.Motor_RatedCurrent.value;
    motor_par->foc_para.motorpar.i_phasevol_v = 24;
    motor_par->foc_para.motorpar.i_poles_n = 14;//2;
    motor_par->foc_para.motorpar.i_rstator_ohm = 1.1;
    motor_par->foc_para.motorpar.i_samplingper_s = 0.00005;
    //motor_par->foc_para.motorpar.func_smc_const(&motor_par->foc_para.motorpar);

    motor_par->foc_para.speedcalpar.i_speedacq = 20;//20
    motor_par->foc_para.speedcalpar.i_speedfilter = 0.1;
    motor_par->foc_para.speedcalpar.i_speedlooptime_s = HPM_MOTOR_MATH_FL_MDF(0.00005*20);//*20
    motor_par->foc_para.speedcalpar.i_motorpar = &motor_par->foc_para.motorpar;
    motor_par->foc_para.speedcalpar.func_getspd = hpm_mcl_bldc_foc_al_speed;

    motor_par->foc_para.currentdpipar.i_kp = 0.5;//0.5
    motor_par->foc_para.currentdpipar.i_ki = 0.02;
    motor_par->foc_para.currentdpipar.i_max = HPM_MOTOR_MATH_FL_MDF(4000);//5000 0k
    motor_par->foc_para.currentdpipar.func_pid = hpm_mcl_bldc_foc_pi_contrl;

    motor_par->foc_para.currentqpipar.i_kp = 0.5;//0.5
    motor_par->foc_para.currentqpipar.i_ki = 0.02;
    motor_par->foc_para.currentqpipar.i_max = HPM_MOTOR_MATH_FL_MDF(7000);//5000 ok
    motor_par->foc_para.currentqpipar.func_pid = hpm_mcl_bldc_foc_pi_contrl;

    motor_par->foc_para.pwmpar.func_spwm = hpm_mcl_bldc_foc_svpwm;
    motor_par->foc_para.pwmpar.i_pwm_reload_max = PWM_RELOAD*0.95;
    motor_par->foc_para.pwmpar.pwmout.func_set_pwm = bldc_foc_pwmset;
    motor_par->foc_para.pwmpar.pwmout.i_pwm_reload = PWM_RELOAD;
    motor_par->foc_para.pwmpar.pwmout.i_motor_id = MOTOR_ID;

    motor_par->foc_para.samplcurpar.func_sampl = hpm_mcl_bldc_foc_current_cal;
    motor_par->foc_para.func_dqsvpwm =  hpm_mcl_bldc_foc_ctrl_dq_to_pwm;
    
    motor_par->adc_trig_event_callback = &motor_highspeed_loop;


}

void pwmv2_duty_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_PWM_REGISTER_UPDATE_TYPE, uint8_t CMP_SOURCE)
{
    pwmv2_cmp_config_t cmp_cfg[2];
    pwmv2_pair_config_t pwm_cfg;

    cmp_cfg[0].cmp = PWM_PRD;
    cmp_cfg[0].enable_half_cmp = false;
    cmp_cfg[0].enable_hrcmp = false;
    cmp_cfg[0].cmp_source = CMP_SOURCE;
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(1);
    cmp_cfg[0].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;
    
    cmp_cfg[1].cmp = PWM_PRD;
    cmp_cfg[1].enable_half_cmp = false;
    cmp_cfg[1].enable_hrcmp = false;
    cmp_cfg[1].cmp_source = CMP_SOURCE;
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(2);
    cmp_cfg[1].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;

    pwm_cfg.pwm[0].enable_output = true;
    pwm_cfg.pwm[0].enable_async_fault = false;
    pwm_cfg.pwm[0].enable_sync_fault = false;
    pwm_cfg.pwm[0].invert_output = false;
    pwm_cfg.pwm[0].enable_four_cmp = false;
    pwm_cfg.pwm[0].update_trigger = CMP_PWM_REGISTER_UPDATE_TYPE;
    pwm_cfg.pwm[0].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
    
    pwm_cfg.pwm[1].enable_output = true;
    pwm_cfg.pwm[1].enable_async_fault = false;
    pwm_cfg.pwm[1].enable_sync_fault = false;
    pwm_cfg.pwm[1].invert_output = false;
    pwm_cfg.pwm[1].enable_four_cmp = false;
    pwm_cfg.pwm[1].update_trigger = CMP_PWM_REGISTER_UPDATE_TYPE;
    pwm_cfg.pwm[1].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;

    pwmv2_disable_counter(ptr, pwm_counter_0);
    pwmv2_reset_counter(ptr, pwm_counter_0);

    pwmv2_shadow_register_unlock(ptr);
    pwmv2_set_shadow_val(ptr, PWMV2_SHADOW_INDEX(0), PWM_PRD, 0, false);
    //pwmv2_set_shadow_val(ptr, PWMV2_SHADOW_INDEX(9), 1, 0, false);
    pwmv2_set_shadow_val(ptr, PWMV2_SHADOW_INDEX(10), 1, 0, false);
    pwmv2_shadow_register_lock(ptr);

    pwmv2_setup_waveform_in_pair(ptr, pwm_channel_0, &pwm_cfg, PWMV2_CMP_INDEX(0), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(3);
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(4);
    pwmv2_setup_waveform_in_pair(ptr, pwm_channel_2, &pwm_cfg, PWMV2_CMP_INDEX(4), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(5);
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(6);
    pwmv2_setup_waveform_in_pair(ptr, pwm_channel_4, &pwm_cfg, PWMV2_CMP_INDEX(8), &cmp_cfg[0], 2);

    pwmv2_counter_select_data_offset_from_shadow_value(BOARD_PMSM0PWM, pwm_counter_0, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(ptr, pwm_counter_0);
    pwmv2_set_reload_update_time(ptr, pwm_counter_0, CMP_PWM_REGISTER_UPDATE_TYPE);

    pwmv2_counter_select_data_offset_from_shadow_value(BOARD_PMSM0PWM, pwm_counter_1, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(ptr, pwm_counter_1);
    pwmv2_set_reload_update_time(ptr, pwm_counter_1, CMP_PWM_REGISTER_UPDATE_TYPE);

    pwmv2_counter_select_data_offset_from_shadow_value(BOARD_PMSM0PWM, pwm_counter_2, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(ptr, pwm_counter_2);
    pwmv2_set_reload_update_time(ptr, pwm_counter_2, CMP_PWM_REGISTER_UPDATE_TYPE);

    //pwmv2_select_cmp_source(ptr, BOARD_BLDCPWM_CMP_TRIG_CMP, CMP_SOURCE, PWMV2_SHADOW_INDEX(9));
    //pwmv2_set_trigout_cmp_index(ptr, BOARD_BLDC_PWM_TRIG_OUT_CHN, BOARD_BLDCPWM_CMP_TRIG_CMP);
    //pwmv2_cmp_select_counter(ptr, BOARD_BLDCPWM_CMP_TRIG_CMP, pwm_counter_0);
    //pwmv2_shadow_register_lock(ptr);

    ///* start counter0,counter1,counter2 */
    //pwmv2_enable_multi_counter_sync(ptr, 0x07);
    //pwmv2_start_pwm_output_sync(ptr, 0x07);
  
}

void pwmv2_trigfor_adc_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint32_t PWM_CNT, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_PWM_REGISTER_UPDATE_TYPE, uint8_t PWM_TRIGOUT_CH_ADC, uint8_t CMP_SOURCE, uint8_t PWM_CH_TRIG_ADC)
{
    
    pwmv2_cmp_config_t cmp_cfg[2];
    pwmv2_pair_config_t pwm_cfg;
    
    pwmv2_shadow_register_unlock(ptr);
    cmp_cfg[0].cmp = PWM_PRD - 1;
    cmp_cfg[0].enable_half_cmp = false;
    cmp_cfg[0].enable_hrcmp = false;
    cmp_cfg[0].cmp_source = CMP_SOURCE;
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(9);
    cmp_cfg[0].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;
    pwmv2_config_cmp(ptr, PWM_CH_TRIG_ADC, &cmp_cfg[0]);

    pwm_cfg.pwm[0].enable_output = true;
    pwm_cfg.pwm[0].enable_async_fault = false;
    pwm_cfg.pwm[0].enable_sync_fault = false;
    pwm_cfg.pwm[0].invert_output = false;
    pwm_cfg.pwm[0].enable_four_cmp = false;
    pwm_cfg.pwm[0].update_trigger = CMP_PWM_REGISTER_UPDATE_TYPE;
    pwm_cfg.pwm[0].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
    pwmv2_config_pwm(ptr, PWM_CH_TRIG_ADC, &pwm_cfg, false);

    pwmv2_select_cmp_source(ptr, PWM_CH_TRIG_ADC, CMP_SOURCE, PWMV2_SHADOW_INDEX(9));
    pwmv2_set_trigout_cmp_index(ptr, PWM_TRIGOUT_CH_ADC, PWM_CH_TRIG_ADC);
    pwmv2_cmp_select_counter(ptr, PWM_CH_TRIG_ADC, PWM_CNT);
    pwmv2_shadow_register_lock(ptr);

    ///* start counter0,counter1,counter2 */
    //pwmv2_enable_multi_counter_sync(ptr, 0x07);
    //pwmv2_start_pwm_output_sync(ptr, 0x07);  
}

void pwmv2_trigfor_currentctrl_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint32_t PWM_CNT, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_PWM_REGISTER_UPDATE_TYPE, uint8_t PWM_TRIGOUT_CH_CUREENTCTRL, uint8_t CMP_SOURCE, uint8_t PWM_CH_TRIG_CUREENTCTRL)
{
    
    pwmv2_cmp_config_t cmp_cfg[2];
    pwmv2_pair_config_t pwm_cfg;
    
    pwmv2_shadow_register_unlock(ptr);
    cmp_cfg[0].cmp = 200;//PWM_PRD;
    cmp_cfg[0].enable_half_cmp = false;
    cmp_cfg[0].enable_hrcmp = false;
    cmp_cfg[0].cmp_source = CMP_SOURCE;
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(10);
    cmp_cfg[0].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;
    pwmv2_config_cmp(ptr, PWM_CH_TRIG_CUREENTCTRL, &cmp_cfg[0]);

    pwm_cfg.pwm[0].enable_output = true;
    pwm_cfg.pwm[0].enable_async_fault = false;
    pwm_cfg.pwm[0].enable_sync_fault = false;
    pwm_cfg.pwm[0].invert_output = false;
    pwm_cfg.pwm[0].enable_four_cmp = false;
    pwm_cfg.pwm[0].update_trigger = CMP_PWM_REGISTER_UPDATE_TYPE;
    pwm_cfg.pwm[0].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
    pwmv2_config_pwm(ptr, PWM_CH_TRIG_CUREENTCTRL, &pwm_cfg, false);

    pwmv2_select_cmp_source(ptr, PWM_CH_TRIG_CUREENTCTRL, CMP_SOURCE, PWMV2_SHADOW_INDEX(10));
    pwmv2_set_trigout_cmp_index(ptr, PWM_TRIGOUT_CH_CUREENTCTRL, PWM_CH_TRIG_CUREENTCTRL);
    pwmv2_cmp_select_counter(ptr, PWM_CH_TRIG_CUREENTCTRL, PWM_CNT);
    pwmv2_shadow_register_lock(ptr);

    /* start counter0,counter1,counter2 */
    pwmv2_enable_multi_counter_sync(ptr, 0x07);
    pwmv2_start_pwm_output_sync(ptr, 0x07); 

}

void pwm_duty_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_COMPARE)
{
    uint8_t cmp_index = BOARD_PMSM0PWM_CMP_INDEX_0;
    pwmv2_cmp_config_t cmp_config[4] = {0};
    pwmv2_pair_config_t pwm_pair_config = {0};
    pwm_output_channel_t pwm_output_ch_cfg;

    pwm_stop_counter(ptr);

    pwm_set_reload(ptr, 0, PWM_PRD);
    pwm_set_start_count(ptr, 0, 0);
  
    cmp_config[0].mode = CMP_COMPARE;
    cmp_config[0].cmp = PWM_PRD + 1;
    cmp_config[0].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;

    cmp_config[1].mode = CMP_COMPARE;
    cmp_config[1].cmp = PWM_PRD + 1;
    cmp_config[1].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;

    pwm_get_default_pwm_pair_config(ptr, &pwm_pair_config);
    pwm_pair_config.pwm[0].enable_output = true;
    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = 100;
    pwm_pair_config.pwm[0].invert_output = false;

    pwm_pair_config.pwm[1].enable_output = true;
    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = 100;
    pwm_pair_config.pwm[1].invert_output = false;

    {  
   /*     if (status_success != pwm_setup_waveform_in_pair(ptr, BOARD_PMSM0_UH_PWM_OUTPIN, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
            printf("failed to setup waveform\n");
            while(1);
        }
        if (status_success != pwm_setup_waveform_in_pair(ptr, BOARD_PMSM0_VH_PWM_OUTPIN, &pwm_pair_config, cmp_index+4, &cmp_config[0], 2)) {
            printf("failed to setup waveform\n");
            while(1);
        }
        if (status_success != pwm_setup_waveform_in_pair(ptr, BOARD_PMSM0_WH_PWM_OUTPIN, &pwm_pair_config, cmp_index+6, &cmp_config[0], 2)) {
            printf("failed to setup waveform\n");
            while(1);
        }
    */
        if (status_success != pwm_setup_waveform_in_pair(ptr, BOARD_PMSM0_UH_PWM_OUTPIN, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
            printf("failed to setup waveform\n");
            while(1);
        }
        if (status_success != pwm_setup_waveform_in_pair(ptr, BOARD_PMSM0_VH_PWM_OUTPIN, &pwm_pair_config, cmp_index-2, &cmp_config[0], 2)) {
            printf("failed to setup waveform\n");
            while(1);
        }
        if (status_success != pwm_setup_waveform_in_pair(ptr, BOARD_PMSM0_WH_PWM_OUTPIN, &pwm_pair_config, cmp_index-4, &cmp_config[0], 2)) {
            printf("failed to setup waveform\n");
            while(1);
        }  
    }
    pwm_enable_output(ptr, BOARD_PMSM0_UH_PWM_OUTPIN);
    pwm_enable_output(ptr, BOARD_PMSM0_UL_PWM_OUTPIN);
    pwm_enable_output(ptr, BOARD_PMSM0_VH_PWM_OUTPIN);
    pwm_enable_output(ptr, BOARD_PMSM0_VL_PWM_OUTPIN);
    pwm_enable_output(ptr, BOARD_PMSM0_WH_PWM_OUTPIN);
    pwm_enable_output(ptr, BOARD_PMSM0_WL_PWM_OUTPIN);

}
void pwm_trigfor_adc_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_COMPARE, uint8_t PWM_CH_TRIG_ADC)
{
    
    pwmv2_cmp_config_t cmp_config[4] = {0};
    pwm_output_channel_t pwm_output_ch_cfg;


    //cmp_config[2].enable_ex_cmp  = false;
    cmp_config[2].enable_half_cmp = false;
    cmp_config[2].enable_hrcmp = false;
    cmp_config[2].mode           = CMP_COMPARE;
    cmp_config[2].cmp = (PWM_PRD - 1);//PWM_PRD/2;
    cmp_config[2].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;
    pwm_config_cmp(ptr, PWM_CH_TRIG_ADC, &cmp_config[2]);

    pwm_output_ch_cfg.cmp_start_index = PWM_CH_TRIG_ADC;
    pwm_output_ch_cfg.cmp_end_index   = PWM_CH_TRIG_ADC;
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(ptr, PWM_CH_TRIG_ADC, &pwm_output_ch_cfg);


}

void pwm_trigfor_currentctrl_init(PWMV2_Type *ptr, uint32_t PWM_PRD, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_COMPARE, uint8_t PWM_CH_TRIG_CURRENTCtrl)
{
    
    pwmv2_cmp_config_t pwm_trig_currentloop = {0};
    pwm_output_channel_t pwm_output_ch_cfg;
 
    memset(&pwm_trig_currentloop, 0x00, sizeof(pwmv2_cmp_config_t));
    //pwm_trig_currentloop.enable_ex_cmp  = false;
    pwm_trig_currentloop.enable_half_cmp = false;
    pwm_trig_currentloop.enable_hrcmp = false;
    pwm_trig_currentloop.mode = CMP_COMPARE;
    pwm_trig_currentloop.cmp = 200;//PWM_PRD/2 + 200;

  
    pwm_trig_currentloop.update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;
    pwm_config_cmp(ptr, PWM_CH_TRIG_CURRENTCtrl, &pwm_trig_currentloop);
  
    pwm_output_ch_cfg.cmp_start_index = PWM_CH_TRIG_CURRENTCtrl;  
    pwm_output_ch_cfg.cmp_end_index   = PWM_CH_TRIG_CURRENTCtrl; 
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(ptr, PWM_CH_TRIG_CURRENTCtrl, &pwm_output_ch_cfg);
    
    pwm_start_counter(ptr);
    pwm_issue_shadow_register_lock_event(ptr);

}



void timer_init(void)
{
    gptmr_channel_config_t config;

    gptmr_channel_get_default_config(BOARD_BLDC_TMR_1MS, &config);
    config.cmp[0] = BOARD_BLDC_TMR_RELOAD;
    config.debug_mode = 0;
    config.reload = BOARD_BLDC_TMR_RELOAD+1;

    gptmr_enable_irq(BOARD_BLDC_TMR_1MS, GPTMR_CH_CMP_IRQ_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP));
    gptmr_channel_config(BOARD_BLDC_TMR_1MS, BOARD_BLDC_TMR_CH, &config, true);
    intc_m_enable_irq_with_priority(BOARD_BLDC_TMR_IRQ, 2);

}
void init_trigger_mux(TRGM_Type * ptr, uint8_t TRAG_INPUT, uint8_t TRAG_INPUT_FOR_ADC)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_same_as_input;//trgm_output_pulse_at_input_rising_edge;
    trgm_output_cfg.input  = TRAG_INPUT;
    trgm_output_config(ptr, TRAG_INPUT_FOR_ADC, &trgm_output_cfg);


}

void init_trigger_cfg(ADC16_Type *ptr, uint8_t trig_ch, uint8_t channel, bool inten, uint32_t ADC_MODULE, uint8_t ADC_PREEMPT_TRIG_LEN)
{
    adc_pmt_config_t pmt_cfg = {0};
        
    pmt_cfg.module = ADC_MODULE;

    {
       pmt_cfg.config.adc16.trig_ch   = trig_ch;
       pmt_cfg.config.adc16.trig_len  = ADC_PREEMPT_TRIG_LEN;
       pmt_cfg.config.adc16.adc_ch[0] = channel;
       pmt_cfg.config.adc16.inten[0] = inten;
       pmt_cfg.adc_base.adc16 = ptr;
    }
    hpm_adc_set_preempt_config(&pmt_cfg);
}

void adc_cfg_init(ADC16_Type *ptr, uint8_t channel, uint32_t sample_cycle, uint32_t ADC_MODULE, uint32_t ADC_TRG)
{
    adc_config_t cfg;
    adc_channel_config_t ch_cfg;
    cfg.module = ADC_MODULE;
    hpm_adc_init_default_config(&cfg);

    cfg.config.adc16.res            = adc16_res_16_bits;
    cfg.config.adc16.conv_mode      = adc16_conv_mode_preemption;
    cfg.config.adc16.adc_clk_div    = adc16_clock_divider_4;//3;
    cfg.config.adc16.sel_sync_ahb   = false;//true;
    cfg.config.adc16.adc_ahb_en = true;
    cfg.adc_base.adc16 = ptr;
    hpm_adc_init(&cfg);


    ch_cfg.module = ADC_MODULE;
    hpm_adc_init_channel_default_config(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle  = sample_cycle;
    ch_cfg.adc_base.adc16                = ptr;
    ch_cfg.config.adc16_ch.ch            = channel;
    hpm_adc_channel_init(&ch_cfg);

   
}


void adc_pins_init(PWMV2_Type *ptr)
{
 
     if  (ptr == HPM_PWM1)
    {
        
      //HPM_IOC->PAD[IOC_PAD_PB10].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;//53 solution
      //HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
      //HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;//53 evk
      //HPM_IOC->PAD[IOC_PAD_PB14].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;

        HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK; //6E evk       
        HPM_IOC->PAD[IOC_PAD_PF08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        
    } 


}





void pwm_pins_init(PWMV2_Type *ptr)
{
   
     if  (ptr == HPM_PWM1)
    {

     
        //HPM_IOC->PAD[IOC_PAD_PA24].FUNC_CTL = IOC_PA24_FUNC_CTL_PWM0_P_0;//53solution
        //HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_PWM0_P_1;
    
        //HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_PWM0_P_4;
        //HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_PWM0_P_5;
        //HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_PWM0_P_6;
        //HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_PWM0_P_7;

        //HPM_IOC->PAD[IOC_PAD_PA26].FUNC_CTL = IOC_PA26_FUNC_CTL_PWM0_P_2;//53evk
        //HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_PWM0_P_3;
        //HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_PWM0_P_4;
        //HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_PWM0_P_5;
        //HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_PWM0_P_6;
        //HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_PWM0_P_7;

        HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;  //6E evk
        HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
        HPM_IOC->PAD[IOC_PAD_PE10].FUNC_CTL = IOC_PE10_FUNC_CTL_PWM1_P_2;
        HPM_IOC->PAD[IOC_PAD_PE11].FUNC_CTL = IOC_PE11_FUNC_CTL_PWM1_P_3;
        HPM_IOC->PAD[IOC_PAD_PE12].FUNC_CTL = IOC_PE12_FUNC_CTL_PWM1_P_4;
        HPM_IOC->PAD[IOC_PAD_PE13].FUNC_CTL = IOC_PE13_FUNC_CTL_PWM1_P_5;
    }
    


}

    




void motor_foc_angle_align(MOTOR_PARA *motor_par,MOTOR_CONTROL_Global* motor_ctrl,uint32_t current_set)
{
   
    //do
   {    
       //if(motor_ctrl->flag_1ms == 1)
        if(motor_ctrl->zero_CW == motor_zero_control)
       {
           //motor_ctrl->flag_1ms = 0;
           motor_par->adc_trig_event_callback = motor_angle_align_loop;   
           enable_all_pwm_output(BOARD_PMSM0PWM);
           motor_ctrl->zero_cnt++;   

           if((motor_ctrl->commu_IdRef < current_set)&&(motor_ctrl->zero_cnt<=5))
              {
                  motor_ctrl->commu_IdRef = 20 + motor_ctrl->commu_IdRef;
                  motor_ctrl->commu_theta = 90;    
                  //printf("zeroctrl  id ++\n");                       
              }
           else 
              {
                  if(motor_ctrl->commu_theta>0)
                  {
                        motor_ctrl->commu_theta =   motor_ctrl->commu_theta - 0.1 ; 
                        //printf("zeroctrl  theta --\n");   
                   
                  }
                  else
                  {
                        if(motor_ctrl->commu_IdRef>0)
                        {
                               motor_ctrl->commu_IdRef = motor_ctrl->commu_IdRef - 10;                         
                        }
                        else
                        { 
                               disable_all_pwm_output(BOARD_PMSM0PWM);     
                               motor_ctrl->motor_status = motor_zero_status;   
                               //paraObj[0].CmdObj.zero_ControlWord.value = 0;
                               qeiv2_cfg_init();
                                motor_par->adc_trig_event_callback = motor_highspeed_loop; 
                                //motor_zero_control = 0; 
                                //motor_ctrl->zero_CW = 0x0;  
                                //printf("zeroctrl  ok\n");             
                            }

                        }
                }
         
      }

    }//while(motor_ctrl->zero_CW == motor_zero_control);
    //printf("zeroctrl  not ok\n");  

}

void qeiv2_pins_init(PWMV2_Type *ptr)
{

    if (ptr == HPM_PWM1)
    {

        //HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_QEI0_A;//53solution
        //HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_QEI0_B;

        //HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_QEI1_A;//53 evk
        //HPM_IOC->PAD[IOC_PAD_PA11].FUNC_CTL = IOC_PA11_FUNC_CTL_QEI1_B;

        HPM_IOC->PAD[IOC_PAD_PB07].FUNC_CTL = IOC_PB07_FUNC_CTL_QEI0_A;//6E evk
        HPM_IOC->PAD[IOC_PAD_PB06].FUNC_CTL = IOC_PB06_FUNC_CTL_QEI0_B;

    }


}


void qeiv2_cfg_init(void)
{
    qeiv2_reset_counter(BOARD_PMSM0_QEI_BASE);

    qeiv2_set_work_mode(BOARD_PMSM0_QEI_BASE, qeiv2_work_mode_abz);

    qeiv2_config_z_phase_counter_mode(BOARD_PMSM0_QEI_BASE, qeiv2_z_count_inc_on_phase_count_max);
    qeiv2_config_phmax_phparam(BOARD_PMSM0_QEI_BASE, BOARD_PMSM0_QEI_FOC_PHASE_COUNT_PER_REV);


    intc_m_enable_irq_with_priority(BOARD_PMSM0_QEI_IRQ, 1);

    qeiv2_set_phcnt_cmp_value(BOARD_PMSM0_QEI_IRQ, 4);

    qeiv2_set_cmp2_match_option(BOARD_PMSM0_QEI_BASE, true, false, true, true, true, true, true);
    qeiv2_enable_load_read_trigger_event(BOARD_PMSM0_QEI_BASE, QEIV2_EVENT_POSITION_COMPARE_FLAG_MASK);


    qeiv2_release_counter(BOARD_PMSM0_QEI_BASE);

}






MOTOR_CONTROL_Global Motor_Control_Global;
CMDGENE_Obj CMDGENEObj;
MOTOR_PARA motor;
qei_CalObj qeiCalObj;


void lv_set_adval_middle(BLDC_CONTROL_FOC_PARA *par, uint32_t ADC_TRG)
{
    
    uint32_t adc_u_sum = 0;
    uint32_t adc_v_sum = 0;
    uint32_t adc_w_sum = 0;
    uint8_t times = 0;
    par->pwmpar.pwmout.pwm_u = PWM_RELOAD >> 1;
    par->pwmpar.pwmout.pwm_v = PWM_RELOAD >> 1;
    par->pwmpar.pwmout.pwm_w = PWM_RELOAD >> 1;

    par->pwmpar.pwmout.func_set_pwm(&par->pwmpar.pwmout);
    /*do{
        if(Motor_Control_Global.flag_1ms == 1)
        {

            adc_u_sum += ((adc_buff[0][ADC_TRG*4]&0xffff)>>4)&0xfff;
            adc_v_sum += ((adc_buff[1][ADC_TRG*4]&0xffff)>>4)&0xfff;
            adc_w_sum += ((adc_buff[2][ADC_TRG*4]&0xffff)>>4)&0xfff;
            times++;
            if(times >= BLDC_CURRENT_SET_TIME_MS){
                break;
            }
            Motor_Control_Global.flag_1ms = 0;
        }
    }while(1);
    par->samplcurpar.adc_u_middle = adc_u_sum/ BLDC_CURRENT_SET_TIME_MS;
    par->samplcurpar.adc_v_middle = adc_v_sum/ BLDC_CURRENT_SET_TIME_MS;
    par->samplcurpar.adc_w_middle = adc_w_sum/ BLDC_CURRENT_SET_TIME_MS;*/
    par->samplcurpar.adc_u_middle = ((adc_buff[0][ADC_TRG*4]&0xffff)>>4)&0xfff;//adc_u_sum/ BLDC_CURRENT_SET_TIME_MS;
    par->samplcurpar.adc_v_middle = ((adc_buff[1][ADC_TRG*4]&0xffff)>>4)&0xfff;//adc_v_sum/ BLDC_CURRENT_SET_TIME_MS;
    par->samplcurpar.adc_w_middle = ((adc_buff[2][ADC_TRG*4]&0xffff)>>4)&0xfff;//adc_w_sum/ BLDC_CURRENT_SET_TIME_MS;
}


void pmsm_motor1_init(void)
{
    adc_pins_init(BOARD_PMSM0PWM);

    pwm_pins_init(BOARD_PMSM0PWM);
    qeiv2_pins_init(BOARD_PMSM0PWM);
    qeiv2_cfg_init();

    //board_init_adc_clock(BOARD_BLDC_ADC_U_BASE, true);
    //board_init_adc_clock(BOARD_BLDC_ADC_V_BASE, true);
    //board_init_adc_clock(BOARD_BLDC_ADC_W_BASE, true);

    adc_cfg_init(BOARD_PMSM0_ADC_U_BASE, BOARD_PMSM0_ADC_CH_U, 15, BOARD_PMSM0_ADC_MODULE, BOARD_PMSM0_ADC_TRG);
    adc_cfg_init(BOARD_PMSM0_ADC_V_BASE, BOARD_PMSM0_ADC_CH_V, 15, BOARD_PMSM0_ADC_MODULE, BOARD_PMSM0_ADC_TRG);

    init_trigger_mux(BOARD_PMSM0PWM_TRGM, BOARD_PMSM0_TRIGMUX_IN_NUM, BOARD_PMSM0_TRG_NUM);
    
    init_trigger_cfg(BOARD_PMSM0_ADC_U_BASE, BOARD_PMSM0_ADC_TRG, BOARD_PMSM0_ADC_CH_U, true, BOARD_PMSM0_ADC_MODULE, BOARD_PMSM0_ADC_PREEMPT_TRIG_LEN);
    init_trigger_cfg(BOARD_PMSM0_ADC_V_BASE, BOARD_PMSM0_ADC_TRG, BOARD_PMSM0_ADC_CH_V, true, BOARD_PMSM0_ADC_MODULE, BOARD_PMSM0_ADC_PREEMPT_TRIG_LEN);

#if BOARD_PMSM0_ADC_MODULE == ADCX_MODULE_ADC16
    adc16_set_pmt_queue_enable(BOARD_PMSM0_ADC_U_BASE, BOARD_PMSM0_ADC_TRG, true);
    adc16_set_pmt_queue_enable(BOARD_PMSM0_ADC_V_BASE, BOARD_PMSM0_ADC_TRG, true);
    //adc16_enable_pmt_queue(BOARD_BLDC_ADC_U_BASE, BOARD_BLDC_ADC_TRG);
    //adc16_enable_pmt_queue(BOARD_BLDC_ADC_V_BASE, BOARD_BLDC_ADC_TRG);
#endif

    adc_type hpm_adc_motor0_a;
    adc_type  hpm_adc_motor0_b;
    adc_module_cfg(&hpm_adc_motor0_a,BOARD_PMSM0_ADC_MODULE,BOARD_PMSM0_ADC_U_BASE);
    adc_module_cfg(&hpm_adc_motor0_b,BOARD_PMSM0_ADC_MODULE,BOARD_PMSM0_ADC_V_BASE);
    hpm_adc_init_pmt_dma(&hpm_adc_motor0_a, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[0]));
    hpm_adc_init_pmt_dma(&hpm_adc_motor0_b, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[1]));

    //pwm_duty_init(BOARD_PMSM0PWM, PWM_RELOAD, pwm_shadow_register_update_on_shlk, pwm_cmp_mode_output_compare);
    pwmv2_duty_init(BOARD_PMSM0PWM, PWM_RELOAD, pwm_shadow_register_update_on_reload, pwm_reload_update_on_reload, cmp_value_from_shadow_val);
    //pwm_trigfor_adc_init(BOARD_PMSM0PWM, PWM_RELOAD, pwm_shadow_register_update_on_shlk, pwm_cmp_mode_output_compare, BOARD_PMSM0_PWM_TRIG_CMP_INDEX);
    pwmv2_trigfor_adc_init(BOARD_PMSM0PWM, PWM_RELOAD, PWM_CNT0, pwm_shadow_register_update_on_reload, pwm_reload_update_on_reload, BOARD_PMSM0_PWM_TRIGOUT_CH_ADC, cmp_value_from_shadow_val, BOARD_PMSM0_PWM_TRIG_CMP_INDEX);
    //pwm_trigfor_currentctrl_init(BOARD_PMSM0PWM, PWM_RELOAD, pwm_shadow_register_update_on_shlk, pwm_cmp_mode_output_compare, BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP);
    pwmv2_trigfor_currentctrl_init(BOARD_PMSM0PWM, PWM_RELOAD, PWM_CNT0, pwm_shadow_register_update_on_reload, pwm_reload_update_on_reload, BOARD_PMSM0_PWM_TRIGOUT_CH_CURRENTLOOP, cmp_value_from_shadow_val, BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP);

   
    motor_param_int(&motor, BLDC_MOTOR0_INDEX);
    pi_param_init(&motor.speedloop_para, 0.03,0.0001,500);
    pi_param_init(&motor.position_para, 0.0005, 0, 50*4000);
    param_init(&Motor_Control_Global,&qeiCalObj,&motor);
    board_delay_ms(1000);//电流采样避免采样不到
    lv_set_adval_middle(&motor.foc_para, BOARD_PMSM0_ADC_TRG);
    
    timer_init();
    //hpm_adc_enable_interrupts(&hpm_adc_motor0_a, BOARD_PMSM0_ADC_TRIG_FLAG);//ADC转换完成中断，测试是否PWM触发ADC的周期、触发时刻正确
    //intc_m_enable_irq_with_priority(BOARD_PMSM0_ADC_IRQn, 6);
}

void isr_adc111(void)
{
    uint32_t status;
    adc_type hpm_adc_motor0_a;
    adc_module_cfg(&hpm_adc_motor0_a,BOARD_PMSM0_ADC_MODULE,BOARD_PMSM0_ADC_U_BASE);//必须指定motor0_a为adc16
    status = hpm_adc_get_status_flags(&hpm_adc_motor0_a);
   
    if ((status & BOARD_PMSM0_ADC_TRIG_FLAG) != 0) {
     gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 1);
        hpm_adc_clear_status_flags(&hpm_adc_motor0_a, BOARD_PMSM0_ADC_TRIG_FLAG);
        gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 0);
    }
    
}
SDK_DECLARE_EXT_ISR_M(BOARD_PMSM0_ADC_IRQn, isr_adc111)






void isr_current_loop(void)
{
    uint32_t status, status1, status1cnt;
    float user_give_angle = 0;
    //status = pwm_get_status(BOARD_PMSM0PWM);
    status = pwmv2_get_cmp_irq_status(BOARD_PMSM0PWM);
     gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 1);
    if (PWM_IRQ_CMP(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP) == (status & PWM_IRQ_CMP(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP))) 
    {
      
        //pwm_clear_status(BOARD_PMSM0PWM, status);
        pwmv2_clear_cmp_irq_status(BOARD_PMSM0PWM, status); 
        motor.adc_trig_event_callback(&motor,&Motor_Control_Global,&qeiCalObj);
        
    }
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 0);
}

SDK_DECLARE_EXT_ISR_M(BOARD_PMSM0APP_PWM_IRQ, isr_current_loop)



void isr_speed_loop(void)
{
  
    volatile uint32_t s = BOARD_BLDC_TMR_1MS->SR;
    BOARD_BLDC_TMR_1MS->SR = s;
//gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 1);
    if (s & GPTMR_CH_CMP_STAT_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP)) 
    {
        Motor_Control_Global.flag_1ms = !Motor_Control_Global.flag_1ms; 
#if MOTORCONTROL_EC_OR_STUDIO
        controlword_update(&paraObj[0],&Motor_Control_Global);
#endif
#if !MOTORCONTROL_EC_OR_STUDIO
    Motor_Control_Global.zero_CW = 0x1210;//((int16_t)paraObj->CmdObj.zero_ControlWord.value & 0xff00) |  (Global->motor_status & 0x00ff);//0x1210;//
    
    if (Motor_Control_Global.motor_status == motor_zero_status)
    {
        Motor_Control_Global.zero_CW = 0;//
    }
#endif
        motor_foc_angle_align(&motor,&Motor_Control_Global,100);


        if(Motor_Control_Global.motor_CW == 1)
        {                     
            if(Motor_Control_Global.OP_mode == POSITION_MODE)
            {   
                positionloop_ctrl(&motor, BOARD_PMSM0PWM, Motor_Control_Global.motor_CW,&CMDGENEObj ,&qeiCalObj );
            }
            else if(Motor_Control_Global.OP_mode == SPEED_MODE)
            {            
                speedloop_ctrl(&motor, BOARD_PMSM0PWM, Motor_Control_Global.motor_CW,&CMDGENEObj );
            }
#if !MOTORCONTROL_EC_OR_STUDIO
 //qei_CalObj* qei_CalHdl;
            motor.position_para.cur = HPM_MOTOR_MATH_FL_MDF(qeiCalObj.pos);  //即使不使能，主站也可以实时观测电机位置
#endif
        }
        else
        {  
            motor.speedloop_para.target = 0;
            motor.speedloop_para.cur = 0;
            motor.speedloop_para.mem = 0;
            motor.position_para.target = 0;
            motor.position_para.cur = 0;
            motor.position_para.mem = 0;
            cmd_gene_disable(&CMDGENEObj.CMDGENE_UserObj); 

        }
#if !MOTORCONTROL_EC_OR_STUDIO
      //qei_CalObj* qei_CalHdl;
      //motor.position_para.cur = HPM_MOTOR_MATH_FL_MDF(qeiCalObj.pos);  //即使不使能，主站也可以实时观测电机位置
#endif

#if MOTORCONTROL_EC_OR_STUDIO
      param_update(&paraObj[0],&CMDGENEObj,&qeiCalObj);  //主站实时更新速度规划信息
#endif
    }
//gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOF, 10, 0);
}
SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_TMR_IRQ, isr_speed_loop)



mcl_user_value_t motor_speed;
mcl_user_value_t motor_position;

void motor_speed_loop_init(void)
{
    if (motor_position.enable) {
        motor_position.enable = false;
    }
    motor_speed.enable = false;//true;
    Motor_Control_Global.motor_CW = 0x00;//0x01;
    Motor_Control_Global.OP_mode = SPEED_MODE;//POSITION_MODE;
    //printf("speedloopinit: %d", Motor_Control_Global.motor_CW);
    //motor0_speed_loop_para_init();
}

void motor_speed_loop_set(int32_t target_speed)
{
    motor_speed.value = (float)target_speed * BOARD_PMSM0_QEI_FOC_PHASE_COUNT_PER_REV / (float)9.2;// * CIA402_SPEED_COEFFICIENT * MCL_2PI;
    CMDGENEObj.CMDGENE_OutObj.VelCmd = motor_speed.value;
    //paraObj[0].VelObj.Vel_Ref.value = (float)target_speed / (float)9.2;
    //hpm_mcl_loop_set_speed(&motor0.loop, motor_speed);
    //printf("speedset: %d", target_speed);
}

void motor_postion_loop_init(void)
{
    if (motor_speed.enable) {
        motor_speed.enable = false;
        //hpm_mcl_loop_set_speed(&motor0.loop, motor_speed);
    }
    motor_position.enable = false;//true;
    Motor_Control_Global.motor_CW = 0x00;//0x01;
    Motor_Control_Global.OP_mode = POSITION_MODE;//SPEED_MODE;
    //printf("positionloopinit: %d", Motor_Control_Global.motor_CW);
    //motor0_position_loop_para_init();
    //hpm_mcl_enable_position_loop(&motor0.loop);
}

void motor_position_loop_set(int32_t target_position)
{
    //motor_position.value = (float)target_position / CIA402_PSITION_COEFFICIENT * MCL_2PI;
    motor_position.value = (float)target_position / CIA402_PSITION_COEFFICIENT * BOARD_PMSM0_QEI_FOC_PHASE_COUNT_PER_REV;
    CMDGENEObj.CMDGENE_OutObj.PosCmd = motor_position.value;
    //hpm_mcl_loop_set_position(&motor0.loop, motor_position);
    //printf("positionset: %d", target_position);
}

int32_t motor_get_actual_speed(void)
{
    //return (int32_t)(motor.speedloop_para.cur * (float)9.2 / (float)4000);// / MCL_2PI / CIA402_SPEED_COEFFICIENT);
    int32_t m_speedbak;
    m_speedbak = (int32_t)(motor.speedloop_para.cur * (float)9.2 / (float)BOARD_PMSM0_QEI_FOC_PHASE_COUNT_PER_REV);
    //printf("speedcur: %d", m_speedbak);
    return m_speedbak;
    
}

int32_t motor_get_actual_position(void)
{
    //float position;
    //hpm_mcl_encoder_get_absolute_theta(&motor0.encoder, &position);
    //printf("positioncur: %f", motor.position_para.cur);
    return (int32_t)(motor.position_para.cur / (float)BOARD_PMSM0_QEI_FOC_PHASE_COUNT_PER_REV * (float)CIA402_PSITION_COEFFICIENT);
    //printf("positioncur: %f", motor.position_para.cur);
}

void motor_stop(void)
{
    if (motor_speed.enable) {
        motor_speed.value = 0;
        CMDGENEObj.CMDGENE_OutObj.VelCmd = motor_speed.value;
        //paraObj[0].VelObj.Vel_Ref.value = motor_speed.value;
        //hpm_mcl_loop_set_speed(&motor0.loop, motor_speed);
    } else if (motor_position.enable) {
        //float position;
        //hpm_mcl_encoder_get_absolute_theta(&motor0.encoder, &position);
        motor_position.value = motor.position_para.cur / (float)BOARD_PMSM0_QEI_FOC_PHASE_COUNT_PER_REV * CIA402_PSITION_COEFFICIENT;
        CMDGENEObj.CMDGENE_OutObj.PosCmd = motor_position.value;
        //hpm_mcl_loop_set_position(&motor0.loop, motor_position);
    }
    Motor_Control_Global.motor_CW = 0x0;
    //printf("stop: %d", Motor_Control_Global.motor_CW);
}

void motor_enable(void)
{
    //enable_all_pwm_output(BOARD_PMSM0PWM);
    //motor_foc_angle_align(&motor,&Motor_Control_Global,100);
    //if(Motor_Control_Global.zero_CW !=  motor_zero_control)
    {
        Motor_Control_Global.motor_CW = 0x01;
        //CMDGENEObj.CMDGENE_OutObj.VelCmd = 10000;//motor_speed.value;
    }
    //printf("ctrl: %d", Motor_Control_Global.motor_CW);
}

void motor_disable(void)
{
    //disable_all_pwm_output(BOARD_PMSM0PWM);
    Motor_Control_Global.motor_CW = 0x0;
    //printf("disable: %d", Motor_Control_Global.motor_CW);
}

