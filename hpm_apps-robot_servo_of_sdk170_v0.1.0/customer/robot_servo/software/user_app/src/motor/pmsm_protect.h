/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef Steppping_Motor_CurrentCtrl_H_
#define Steppping_Motor_CurrentCtrl_H_



#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "hpm_debug_console.h"
#include "hpm_sysctl_drv.h"
#include "Parm_Global.h"
#include "pmsm_currentctrl.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_adc.h"
#include "hpm_gpio_drv.h"
#include "pmsm_define.h"
#include "hpm_qeiv2_drv.h"


typedef struct {
    uint16_t maxvoltage;  //过压阈值
    uint16_t minvoltage;  //欠压阈值
    uint16_t times_vol;  //母线电压检测周期
    uint16_t maxcurrent;
    uint16_t times_current;
    uint16_t maxtemp;
    uint16_t mintemp;
    uint16_t times_temp;
    float maxmin_speed;
    uint16_t times_speed;
    uint16_t times_encoder;
    uint16_t errorword;

}MOTOR_CONTROL_PROTECT;

extern MOTOR_CONTROL_PROTECT Motor_Control_Protect;
extern void motor_protect(MOTOR_PARA *par, MOTOR_CONTROL_Global* global);





#endif
