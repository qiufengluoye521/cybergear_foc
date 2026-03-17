/*
 * Copyright (c) 2023 hpmicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef DATA_TRANS_H
#define DATA_TRANS_H

//#include "package_deal.h"

#define PACKET_DEALDATA_IDX                     (5)
#define COMPEN_HEAD_FRAME_LEN                   (4)
#define DEAL_LEN_MIN                            (6)
#define ACK                                     (0x06)
#define ACK_EER                                 (0x07)
#define NAK                                     (0x15)
#define NODE_MAX                                (4)
#define NODE_IDX                                (0)
#define CMD_IDX                                 (1)
#define ADDR_H_IDX                              (2)
#define ADDR_L_IDX                              (3)
#define BYTE_NUM_H_IDX                          (4)
#define BYTE_NUM_L_IDX                          (5)
#define BANK_IDX                                (2)
#define MODE_IDX                                (3)
#define DATA_NUM_H_IDX                          (4)
#define DATA_NUM_L_IDX                          (5)
#define MODE_LEN                                (4)

/**
 * @brief     enum of Data frame's command type.
 */
typedef enum _tag_CmdType{
    Single_WRITE ,/**< signle data write. */
    Single_READ ,/**< signle data read. */
    Seq_WRITE ,/**< sequence data write. */
    Seq_READ ,/**< sequence data read. */
}CmdType;

/**
 * @brief     struct of parameter.
 */
typedef  struct _tag_para_typ_Obj{
  float value;/**< para value. */
  int16_t index;/**< para index. */
  uint8_t dotbit;/**< para dotbit. */
}ParaTyp_Obj; 

/**
 * @brief     parameters of the velocity loop.
 */
typedef struct _tag_Vel_Cfg_Obj{
ParaTyp_Obj Kp;/**< proportional gain. */
ParaTyp_Obj Ki;/**< integral gain. */
ParaTyp_Obj Vel_ForWard_Gain;/**< speed feedforward gain. */
ParaTyp_Obj LowPassWn;/**< low pass wn freq. */
ParaTyp_Obj Vel_Ref;/**< velocity cmd. */
ParaTyp_Obj Acc_Ref;/**< acceleration cmd. */
ParaTyp_Obj Jerk_Ref;/**< jerk cmd. */
ParaTyp_Obj Dec_Ref;/**< dec cmd. */
ParaTyp_Obj Vel_Fdk;/**< velocity feedback. */
ParaTyp_Obj Vel_err;/**< velocity error. */
ParaTyp_Obj Vel_IntrErr;/**< velocity error(counts). */
ParaTyp_Obj Vel_IntrFdk;/**< velocity feedback(counts). */
ParaTyp_Obj Vel_IntrCmd;/**< velocity cmd(counts). */
ParaTyp_Obj VelCnstTime;/**< velocity const time. */
}VelCfg_Obj;

/**
 * @brief     parameters of the current loop.
 */
typedef struct _tag_Current_Cfg_Obj{
ParaTyp_Obj Kp;/**< proportional gain. */
ParaTyp_Obj Ki;/**< integral gain. */
ParaTyp_Obj Current_ref;/**< current cmd. */
ParaTyp_Obj Current_Fdk;/**< current feedback. */
}CurrentCfg_Obj;

/**
 * @brief     parameters of motor .
 */
typedef struct _tag_MotorPara_Obj{
ParaTyp_Obj Motor_RatedCurrent;/**< rated current. */
ParaTyp_Obj Mortor_RatedVlotage;/**< rated vlotage. */
ParaTyp_Obj Mortor_R;/**< R. */
ParaTyp_Obj Mortor_L;/**< L. */
ParaTyp_Obj Mortor_PoleNum;/**< pole num. */
ParaTyp_Obj Mortor_PolePicth;/**< pole picth. */
ParaTyp_Obj Motor_Type;/**< motor type. */
ParaTyp_Obj Motor_EncType;/**< encoder type. */
ParaTyp_Obj Motor_PlusCnt;/**< pulse cnts per rotation of the motor . */
}MotorPara_Obj;

/**
 * @brief     parameters of motor cfg .
 */
typedef struct _tag_Motor_Cfg_Obj{
MotorPara_Obj  MotorParaObj;
}MotorCfg_Obj;

/**
 * @brief     parameters of scope func .
 */
typedef struct _tag_Scope_Obj{
ParaTyp_Obj Scp_SampeCnt;/**< sample count . */
ParaTyp_Obj Scp_SampleGap;/**< sample interval . */
ParaTyp_Obj Scp_channelCfg1;/**< sample channel 1 configuration . */
ParaTyp_Obj Scp_channelCfg2;/**< sample channel 2 configuration . */
ParaTyp_Obj Scp_channelCfg3;/**< sample channel 3 configuration . */
ParaTyp_Obj Scp_channelCfg4;/**< sample channel 4 configuration . */
ParaTyp_Obj Scp_channelCfg5;/**< sample channel 5 configuration . */
ParaTyp_Obj Scp_channelCfg6;/**< sample channel 6 configuration . */
}Scope_Obj;

/**
 * @brief     parameters of the position loop .
 */
typedef struct _tag_PosCfg_Obj{
ParaTyp_Obj Kp;/**< proportional gain. */
ParaTyp_Obj Pos_ref;/**< position cmd. */
ParaTyp_Obj Pos_Fdk;/**< position feedback. */
ParaTyp_Obj Pos_err;/**< position error. */
ParaTyp_Obj Pos_IntrErr;/**< position error(counts). */
ParaTyp_Obj Pos_IntrFdk;/**< position feedback(counts). */
ParaTyp_Obj Pos_IntrCmd;/**< position cmd(counts). */
ParaTyp_Obj CycleType;/**< motion type. */
ParaTyp_Obj CmdType;/**< cmd generation mode. */
ParaTyp_Obj DwellTime;/**< waiting time. */
ParaTyp_Obj CycleCount;/**< cycle time. */
}PosCfg_Obj;

/**
 * @brief     parameters of cmd .
 */
typedef struct _tag_Cmd_Obj{
ParaTyp_Obj ControlWord;
ParaTyp_Obj OpMode;
ParaTyp_Obj zero_ControlWord;
ParaTyp_Obj status;
}Cmd_Obj;


/**
 * @brief     struct of parameters .
 */
typedef struct _tag_Para_Obj{
 MotorCfg_Obj MotorObj;
 CurrentCfg_Obj CurrentObj;
 VelCfg_Obj VelObj;
 PosCfg_Obj PosObj;
 Scope_Obj  ScopeObj;
 Cmd_Obj   CmdObj;
}Para_Obj;


/**
 * @brief           find a pointer to the struct  by address info
 * @param[in]       address       para index.
 * @param[in,out]   paraHdl       para struct to operate on.
 * @returns         a pointer to para struct
 */
ParaTyp_Obj* para_index_map(int16_t address,Para_Obj* paraHdl);

#endif 