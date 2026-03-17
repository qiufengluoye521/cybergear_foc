/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef Steppping_Motor_SpeedCtrl_H_
#define Steppping_Motor_SpeedCtrl_H_

#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "hpm_debug_console.h"
#include "hpm_sysctl_drv.h"
#include "Parm_Global.h"
#include "CMD_GENE.h"

/**
 * @brief PMSM SPEEDCTRL API ;速度控制接口函数
 * @addtogroup PMSM_SPEEDTCTRL_API
 * @{
 *
 */

/***********************************************************************************************************************
 *
 * Definitions
 *
 **********************************************************************************************************************/

/**
 * @brief     Operation mode
 *            ;运行模式
 */
typedef enum _tag_opmode{
POSITION_MODE,
SPEED_MODE,
}OP_MODE;

/**
 * @brief     Speedloop ctrl
 *            ;速度环控制
 * @param[in]    par    motor  param;电机参数结构体
 * @param[in]    ptr    pwm  param;pwm结构体
 * @param[in]    Motor_COntrol_Word    controlword;运行控制指令
 * @param[in]    CMDGENEObj    cmd struct;指令规划结构体
 */
void speedloop_ctrl(MOTOR_PARA *par, PWMV2_Type *ptr, uint8_t Motor_COntrol_Word,CMDGENE_Obj* CMDGENEObj );
/**
 * @brief     Positionloop ctrlt
 *            ;位置环控制
 * @param[in]    par    motor  param;电机参数结构体
 * @param[in]    ptr    pwm  param;pwm结构体
 * @param[in]    Motor_COntrol_Word    controlword;运行控制指令
 * @param[in]    CMDGENEObj    cmd struct;指令规划结构体
 * @param[in]    qei_CalHdl    encoder struct;编码器结构体
 */
void positionloop_ctrl(MOTOR_PARA *par, PWMV2_Type *ptr,uint8_t Motor_COntrol_Word,CMDGENE_Obj* CMDGENEObj,qei_CalObj* qei_CalHdl );

/** @} */

#endif //__PMSM_SPEEDTCTRL_H