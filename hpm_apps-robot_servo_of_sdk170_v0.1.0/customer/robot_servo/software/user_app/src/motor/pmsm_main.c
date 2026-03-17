/**************************************************************************
 * File     : pmsm_main.c
 * Copyright: HPMicro
 * Des      : this can be used be main.c when user project unuses ethercat
 * Data     : 2024-11
 * Author   : 上海先楫半导体科技有限公司 
 * Log      : 
 ***************************************************************************/
#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "hpm_debug_console.h"
#include "hpm_sysctl_drv.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "bldc_foc_cfg.h"
#include "hpm_synt_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_adc.h"
#include "parm_global.h"
#include "pmsm_init.h"
#include "pmsm_currentctrl.h"
#include "libhpm_motor.h"
#include "hpm_qeiv2_drv.h"

extern MOTOR_CONTROL_Global Motor_Control_Global;

void motor_function_init(void)
{ 

    monitor_init();             /* Config hpm monitor init */
    pmsm_motor1_init();         /* Config pmsm init, include pwm, adc, encoder, current_ctrl isr, 1ms_isr, pamsm params*/

    intc_m_enable_irq_with_priority(BOARD_PMSM0APP_PWM_IRQ, 6);//1);         /* Enable current_ctrl isr*/
    pwmv2_enable_cmp_irq(BOARD_PMSM0PWM, (BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP));
  
}
