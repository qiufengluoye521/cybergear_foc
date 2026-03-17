/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef CMDGENE_H_
#define CMDGENE_H_
#include <stdio.h>
#include <math.h>
#include <string.h>

/**
 * @brief CMD GENE API ;指令生成接口函数
 * @addtogroup CMD_GENE_API
 * @{
 *
 */

/***********************************************************************************************************************
 *
 * Definitions
 *
 **********************************************************************************************************************/

/**
 * @brief     Instruction planning output
 *            ;指令规划生成输出
 */
typedef struct _tag_CMDGENE_OUTPUT_Obj
{
        /** @brief Position command r    
        *         ;位移指令 r */
	double PosCmd;
        /** @brief Velocity command    
        *         ;速度指令 r/s */
	double VelCmd;
        /** @brief Acceleration command  
        *         ;加速度指令 r/s/s */
	double AccCmd;
        /** @brief Jerk command 
        *         ;加加速度指令 r/s/s/s */
        double JerkCmd;
}CMDGENE_OUTPUT_Obj;


/**
 * @brief     position info for Instruction planning
 *            ;指令规划位置信息
 */
typedef struct _tag_CMDGENE_POSCFG_Obj{
        /** @brief Start position    
        *         ;起始位置 */
        double  q0;
        /** @brief End position    
        *         ;结束位置 */
        double  q1;
        /** @brief Start velocity    
        *         ;起始速度 */
        double  v0;
        /** @brief End velocity    
        *         ;结束速度 */
        double  v1;
        /** @brief Max velocity    
        *         ;最大速度 */
        double  vmax;
        /** @brief Max acceleration    
        *         ;最大加速度 */
        double  amax;
        /** @brief Max jerk    
        *         ;最大加加速度 */
        double  jmax;
        /** @brief Cycle type    
        *         ;运动类型 */
        unsigned long CycleType;
        /** @brief Cycle count    
        *         ;往返次数 */
	unsigned long CycleCnt;
        /** @brief Dwell time ms    
        *         ;等待时间 ms */
	double DwellTime;
}CMDGENE_POSCFG_Obj;

/**
 * @brief     velocity info for Instruction planning
 *            ;指令规划速度信息
 */
typedef struct _tag_CMDGENE_VELCFG_Obj{
        /** @brief Start position    
        *         ;起始位置 */
        double  q0;
        /** @brief Time spent at a constant speed    
        *         ;匀速时间 */
        double  Tv;
        /** @brief End velocity    
        *         ;起始速度 */
        double  v0;
        /** @brief End velocity    
        *         ;结束速度 */
        double  v1;
        /** @brief Max velocity    
        *         ;最大速度 */
        double  vmax;
        /** @brief Max acceleration    
        *         ;最大加速度 */
        double  amax;
        /** @brief Max jerk    
        *         ;最大加加速度 */
        double  jmax;
}CMDGENE_VELCFG_Obj;

/**
 * @brief     position&velocity info for Instruction planning
 *            ;指令生成信息
 */
typedef struct _tag_CMDGENE_INPUT_Obj
{
        /** @brief position cmd generation config    
        *         ;指令生成位置配置 */
	CMDGENE_POSCFG_Obj CMDGENE_PosCfgObj;
        /** @brief velocity cmd generation config    
        *         ;指令生成速度配置 */
        CMDGENE_VELCFG_Obj CMDGENE_VelCfgObj;
}CMDGENE_INPUT_Obj;

/**
 * @brief     Cmd generation position config
 *            ;指令规划生成中间变量
 */
typedef struct _tag_CMDGENE_USER_Obj
{
	double Ta;
        double Tv;
        double Td;
        double Tj1;
        double Tj2;
        double q_0;
        double q_1;
        double v_0;
        double v_1;
        double vlim;
        double a_max;
        double a_min;
        double a_lima;
        double a_limd;
        double j_max;
        double j_min;
        double t;
        double t_old;
        double t_total;
        double t_timer;
        double sigma;
        double sigma1;
        unsigned long halfcycleCnt;
        unsigned long CycleType;
	    unsigned long CycleCnt;
	    unsigned long DwellTime;
}CMDGENE_USER_Obj;

/**
 * @brief     Cmd generation struct
 *            ;指令规划结构体
 */
typedef struct _tag_CMDGENE_Obj{
        /** @brief Cmd generation input struct    
        *         ;指令生成输入结构体 */
       CMDGENE_INPUT_Obj  CMDGENE_InObj;
       /** @brief Cmd generation user struct    
        *         ;指令生成中间变量结构体 */
       CMDGENE_USER_Obj   CMDGENE_UserObj;
       /** @brief Cmd generation output struct    
        *         ;指令生成输出结构体 */
       CMDGENE_OUTPUT_Obj CMDGENE_OutObj;
}CMDGENE_Obj;

/**
 * @brief     Cycle type enumeration
 *            ;运动类型枚举
 */
typedef enum _tag_emCycleTyp
{
        /** @brief InfiniteMove    
        *         ;连续运动 */
        InfiniteMove,
        /** @brief SingleMove    
        *         ;单次运动 */
	SingleMove,
        /** @brief FiniteMove    
        *         ;多次运动 */
	FiniteMove,
	
	
}emCycleTyp;

/**
 * @brief           Init temp variable while motor stops running
 *                  ;停止运动中间变量初始化
 * @param[in,out]   cmdUserHdl   user struct to operate on.
 */
extern void CMDGENE_DISABLE( CMDGENE_USER_Obj* cmdUserHdl);
/**
 * @brief           Generate Cmd Output with specific end position
 *                  ;没有特定末端位置的指令生成
 * @param[in,out]   CMDGENE_Hdl   struct to operate on.
 */
void CMD_PosCmdGENE(CMDGENE_Obj* CMDGENE_Hdl);
/**
 * @brief           Generate Cmd Output without specific end position
 *                  ;有特定末端位置的指令生成
 * @param[in,out]   CMDGENE_Hdl   struct to operate on.
 */
void CMD_VelCmdGENE(CMDGENE_Obj* CMDGENE_Hdl);

/** @} */

#endif //__CMD_GENE_H


