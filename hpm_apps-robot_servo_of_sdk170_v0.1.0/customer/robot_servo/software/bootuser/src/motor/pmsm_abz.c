/*
 * Copyright (c) 2021 hpmicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "hpm_debug_console.h"
#include "hpm_sysctl_drv.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
//#include "hpm_qei_drv.h"
#include "hpm_gptmr_drv.h"
//#include "hpm_adc12_drv.h"

#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "bldc_foc_cfg.h"
#include "hpm_synt_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_adc.h"
#include "parm_global.h"
#include "pmsm_init.h"
#include "pmsm_currentctrl.h"
#include "cmd_gene.h"

#include "hpm_qeiv2_drv.h"

extern MOTOR_CONTROL_Global Motor_Control_Global;

//int main(void)
void motor_function_init(void)
{
   
    //board_init();
//HPM_IOC->PAD[IOC_PAD_PB25].FUNC_CTL = IOC_PB25_FUNC_CTL_GPIO_B_25;
//gpio_set_pin_output(HPM_GPIO0, GPIO_DO_GPIOB, 25);

/*HPM_IOC->PAD[IOC_PAD_PF10].FUNC_CTL = IOC_PF10_FUNC_CTL_GPIO_F_10;
gpio_set_pin_output(HPM_GPIO0, GPIO_DO_GPIOF, 10);*/

    //HPM_IOC->PAD[IOC_PAD_PE15].FUNC_CTL = IOC_PE15_FUNC_CTL_GPIO_E_15;
    //gpio_set_pin_output(BOARD_LED_GPIO_CTRL, BOARD_LED_GPIO_INDEX, BOARD_LED_GPIO_PIN);
monitor_init();
    pmsm_motor1_init();

    intc_m_enable_irq_with_priority(BOARD_PMSM0APP_PWM_IRQ, 1);
    pwmv2_enable_cmp_irq(BOARD_PMSM0PWM, (BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP));
    
   
    //motor_angle_align();
    //board_delay_ms(1000);

    /*while(1)
    {      
    //gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOE, 15, 1);
    //board_delay_ms(100);
        //board_delay_us(10);
        debug_control();   
        //gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOE, 15, 0);
        //board_delay_ms(100);    
    }*/

   
}
