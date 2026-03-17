/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef Motor_Init_H_
#define Motor_Init_H_

#include "board.h"

#include "hpm_adc.h"
#include "parm_global.h"


/**
 * @brief PMSM INIT API ;电机初始化接口函数
 * @addtogroup PMSM_INIT_API
 * @{
 *
 */

/***********************************************************************************************************************
 *
 * Definitions
 *
 **********************************************************************************************************************/
/**
 * @brief pwm output channel config
 *
 */
typedef struct pwm_output_channel {
    uint8_t cmp_start_index; /**< output channel compare start index */
    uint8_t cmp_end_index;   /**< output channel compare end index */
    bool invert_output;      /**< invert output */
} pwm_output_channel_t;


/**
 * @brief     BLDC current set time
 *            ;电流采样时间设置
 */
#define BLDC_CURRENT_SET_TIME_MS    (200U)
extern uint32_t adc_buff[3][BOARD_BLDC_ADC_SEQ_DMA_SIZE_IN_4BYTES];

/**
 * @brief     Parameter initialization
 *            ;参数初始化
 * @param[in]    global    global struct;全局变量结构体
 * @param[in]    qei_CalHdl    encoder param;编码器参数
 * @param[in]    motor    motor param;电机控制结构体
 */
void param_init(MOTOR_CONTROL_Global* global,qei_CalObj* qeiCalHdl,MOTOR_PARA* motor);


/**
 * @brief     Motor foc angel align
 *            ;预定位操作，给定d轴电流，拉动电机轴，使d轴与A轴重合
 * @param[in]    motor_par    motor  param;电机控制结构体
 * @param[in]    motor_ctrl    global  param;全局变量结构体
 * @param[in]    current_set    current value for d axis;d轴电流
 */
void motor_foc_angle_align(MOTOR_PARA *motor_par,MOTOR_CONTROL_Global* motor_ctrl,uint32_t current_set);

/**
 * @brief     PMSM Motor1 initialization
 *            ;电机初始化
 */
void pmsm_motor1_init(void);


/** @} */

#endif //PMSM_INIT_H_