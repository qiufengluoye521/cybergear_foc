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
//#include "hpm_qei_drv.h"
#include "hpm_qeiv2_drv.h"
#include "hpm_spi_drv.h"


extern MOTOR_CONTROL_Global Motor_Control_Global;
extern MOTOR_PARA motor;
extern qei_CalObj qeiCalObj;

/**
 * @brief     Encoder parameter calculation
 *            ;spi编码器参数计算
 */
typedef struct {
    int32_t init_angle;
    int32_t ph;
    int32_t maxph;
    uint8_t motor_pole;
    int32_t elec_angle;
    int32_t pos;
}spi_CalObj;



extern void electric_angle_cal(qei_CalObj* qei_CalHdl,MOTOR_CONTROL_Global* global);





#endif