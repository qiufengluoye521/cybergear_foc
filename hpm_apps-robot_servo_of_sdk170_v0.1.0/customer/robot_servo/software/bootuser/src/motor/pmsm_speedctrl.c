#include "pmsm_speedctrl.h"
#include "hpm_csr_regs.h"
#include "hpm_gptmr_drv.h"
//#include "hpm_qei_drv.h"
#include "hpm_gpio_drv.h"
#include "parm_global.h"

#include "cmd_gene.h"
#include "hpm_qeiv2_drv.h"


void speedloop_ctrl(MOTOR_PARA *par, PWMV2_Type *ptr, uint8_t Motor_COntrol_Word,CMDGENE_Obj* CMDGENEObj )
{
    enable_all_pwm_output(ptr);//etehrcat csv模式的速度指令由主站下发
#if  MOTORCONTROL_EC_OR_STUDIO      
      CMD_VelCmdGENE(CMDGENEObj);
#endif
#if !MOTORCONTROL_EC_OR_STUDIO
      qei_CalObj* qei_CalHdl;
      par->position_para.cur = HPM_MOTOR_MATH_FL_MDF(qei_CalHdl->pos);
#endif
      par->speedloop_para.target = HPM_MOTOR_MATH_FL_MDF(CMDGENEObj->CMDGENE_OutObj.VelCmd);
      par->speedloop_para.cur = par->foc_para.speedcalpar.o_speedout;
      par->speedloop_para.func_pid(&par->speedloop_para);
      par->foc_para.currentqpipar.target = par->speedloop_para.outval;
      par->foc_para.currentdpipar.target =  0;  

}

void positionloop_ctrl(MOTOR_PARA *par, PWMV2_Type *ptr,uint8_t Motor_COntrol_Word,CMDGENE_Obj* CMDGENEObj,qei_CalObj* qei_CalHdl )
{

      enable_all_pwm_output(ptr); 
     
 #if  MOTORCONTROL_EC_OR_STUDIO     
      CMD_PosCmdGENE(CMDGENEObj);
 #endif
#if !MOTORCONTROL_EC_OR_STUDIO
      par->speedloop_para.cur = par->foc_para.speedcalpar.o_speedout;
#endif
      par->position_para.cur = HPM_MOTOR_MATH_FL_MDF(qei_CalHdl->pos);
      par->position_para.target = HPM_MOTOR_MATH_FL_MDF( CMDGENEObj->CMDGENE_OutObj.PosCmd);
      par->position_para.func_pid(&par->position_para);         
      par->speedloop_para.target =   HPM_MOTOR_MATH_MDF_FL(par->position_para.outval*1000) + CMDGENEObj->CMDGENE_OutObj.VelCmd;
      par->speedloop_para.cur = par->foc_para.speedcalpar.o_speedout;
      par->speedloop_para.func_pid(&par->speedloop_para);
      par->foc_para.currentqpipar.target =  par->speedloop_para.outval;
      par->foc_para.currentdpipar.target =  0;

}



