
/**************************************************************************
 * File     : pmsm_encoder.c
 * Copyright: HPMicro
 * Des      : 
 * Data     : 2024-11
 * Author   : 上海先楫半导体科技有限公司 
 * Log      : 
 ***************************************************************************/
#include "pmsm_encoder.h"

/**********************************************
 * *void motor_encoder_spi_elecinit()
 * * spi encoder init angle
 * *@Input: motor control global structure
 * *#Output：None
 * ***************************************/
void motor_encoder_spi_elecinit(MOTOR_CONTROL_Global* global)
{
    spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};
    spi_control_config_t control_config = {0};

    uint16_t wbuff[1] = {0x3FFF};
    uint16_t rbuff[1] = {0};
    uint16_t m_pos = 0;

    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
    control_config.master_config.addr_enable = false; /* address phase control for master */
    control_config.common_config.trans_mode = spi_trans_write_read_together;//spi_trans_write_read_together;
    spi_transfer(BOARD_APP_SPI_BASE, &control_config, NULL, NULL, (uint16_t *)wbuff, ARRAY_SIZE(wbuff), (uint16_t *)rbuff, ARRAY_SIZE(rbuff));
 
    m_pos = rbuff[0] & 0x3FFF;
    global->Motor_ElecInit = (float)(m_pos);//(float)(m_pos % 1170) / (float)1170 * 360.0;
    uint16_t test;
    test = global->Motor_ElecInit;
}
/**********************************************
 * *void motor_encoder_abz()
 * * abz encoder: read position, cal speed and electrical angle
 * *@Input: QEI structure, motor control global structure
 * *#Output：None
 * ***************************************/
void motor_encoder_abz(qei_CalObj* qei_CalHdl, MOTOR_CONTROL_Global* global)
{
      

     qei_CalHdl->z =qeiv2_get_current_count(BOARD_PMSM0_QEI_BASE, qeiv2_counter_type_z)&0x1fffff;
     qei_CalHdl->ph = qeiv2_get_current_count(BOARD_PMSM0_QEI_BASE, qeiv2_counter_type_phase)&0x1fffff;
     if(qei_CalHdl->z >= (0x200000 >> 1))
     {
        qei_CalHdl->pos = -(((qei_CalHdl->z - 0x200000)*qei_CalHdl->maxph)+qei_CalHdl->ph);
     }
     else
     {
        qei_CalHdl->pos =  -((qei_CalHdl->z*qei_CalHdl->maxph)+qei_CalHdl->ph);
     }
     uint32_t enc_cnt = qei_CalHdl->maxph/qei_CalHdl->motor_pole;
     
     qei_CalHdl->elec_angle = 360- (qei_CalHdl->ph%enc_cnt)*360.0/enc_cnt;

     motor.foc_para.speedcalpar.speedtheta = qei_CalHdl->pos;
     motor.foc_para.electric_angle = qei_CalHdl->elec_angle;
}
/**********************************************
 * *void motor_encoder_spi()
 * * spi encoder: read postion, cal speed and electrical angle
 * *@Input: motor control global structure
 * *#Output：None
 * ***************************************/
void motor_encoder_spi(MOTOR_CONTROL_Global* global)
{
    spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};
    spi_control_config_t control_config = {0};

    uint16_t wbuff[1] = {0x3FFF};
    uint16_t rbuff[1] = {0};
    static uint16_t m_pos = 0;
    static uint16_t m_poslast = 0; 
    static float m_pos_cal = 0;
    static float m_pos_callast = 0; 
    static float m_pos_to_ctrl = 0;
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
    control_config.master_config.addr_enable = false; /* address phase control for master */
    control_config.common_config.trans_mode = spi_trans_write_read_together;//spi_trans_write_read_together;
    spi_transfer(BOARD_APP_SPI_BASE, &control_config, NULL, NULL, (uint16_t *)wbuff, ARRAY_SIZE(wbuff), (uint16_t *)rbuff, ARRAY_SIZE(rbuff));

    m_pos = rbuff[0] & 0x3FFF;
    m_pos_cal = (float)m_pos;
    uint32_t enc_cnt = BOARD_PMSM0_SPI_FOC_PHASE_COUNT_PER_REV/motor.foc_para.motorpar.i_poles_n;
    float m_cnt = enc_cnt;
    if((m_pos_cal - global->Motor_ElecInit) < -m_cnt)
    {
        m_pos_to_ctrl = m_pos_to_ctrl + (m_pos_cal - global->Motor_ElecInit) + BOARD_PMSM0_SPI_FOC_PHASE_COUNT_PER_REV;
    }
    else if ((m_pos_cal - global->Motor_ElecInit) > m_cnt)
    {
        m_pos_to_ctrl = m_pos_to_ctrl + BOARD_PMSM0_SPI_FOC_PHASE_COUNT_PER_REV - (m_pos_cal - global->Motor_ElecInit);
    }
    else
    {
        m_pos_to_ctrl = m_pos_to_ctrl + (m_pos_cal - global->Motor_ElecInit);
    }

    motor.foc_para.speedcalpar.speedtheta = m_pos_to_ctrl;//motor.foc_para.speedcalpar.speedtheta - global->Motor_ElecInit;
    motor.foc_para.electric_angle =  (float)(((m_pos % enc_cnt)) * 360.0 / enc_cnt - 200.6);
    if (motor.foc_para.electric_angle > 360)
    {
        motor.foc_para.electric_angle = fmodf(motor.foc_para.electric_angle, 360);
    }
    else if(motor.foc_para.electric_angle < 0)
    {
        motor.foc_para.electric_angle = fmodf(motor.foc_para.electric_angle, 360) + 360;
    }
    
    //m_poslast = m_pos;
    global->Motor_ElecInit = m_pos;
    m_pos_callast = (float)m_poslast;
}
/**********************************************
 * *void electric_angle_cal()
 * * choose abz or spi for encoder, and reflesh position
 * *@Input: QEI structure, motor control global structure
 * *#Output：None
 * ***************************************/
void electric_angle_cal(qei_CalObj* qei_CalHdl, MOTOR_CONTROL_Global* global)
{
     if (Motor_Control_Global.Motor_Encoder_Type == BOARD_PMSM0_ENCODER_TYPE)//1)//  
     {
          motor_encoder_spi(global);
     }
     else
     {
          motor_encoder_abz(qei_CalHdl, global);
     }
}