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

/**
 * @brief PMSM CURRENTCTRL API ;电流环控制接口函数
 * @addtogroup PMSM_CURRENTCTRL_API
 * @{
 *
 */

/***********************************************************************************************************************
 *
 * Definitions
 *
 **********************************************************************************************************************/

/**
 * @brief     calculate electrical Angle from by encoder position
 *            ;电气角度计算
 * @param[in]    qei_CalHdl    encoder param;编码器参数
 * @param[in]    global    global struct;全局变量结构体
 */
void electric_angle_cal( qei_CalObj* qei_CalHdl,MOTOR_CONTROL_Global* global);
/**
 * @brief     Config PWM  and electrical Angle while looking for initial phase angle
 *            ;配置电角度以及pwm占空比执行预定位操作
 * @param[in]    motor    motor param;电机控制结构体
 * @param[in]    global    global struct;全局变量结构体
 * @param[in]    qei_CalHdl    encoder param;编码器参数
 */
void motor_angle_align_loop(MOTOR_PARA* motor,MOTOR_CONTROL_Global* global,qei_CalObj* qei_CalHdl);
/**
 * @brief     current loop process
 *            ;电流环运行
 * @param[in]    motor    motor param;电机控制结构体
 * @param[in]    global    global struct;全局变量结构体
 * @param[in]    qei_CalHdl    encoder param;编码器参数
 */
void motor_highspeed_loop(MOTOR_PARA* motor,MOTOR_CONTROL_Global* global,qei_CalObj* qei_CalHdl);

/** @} */

#endif //__PMSM_CURRENTCTRL_H