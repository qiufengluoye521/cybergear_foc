/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include "board.h"
//#include "foe.h"

#include "ecat_def.h"
#include "ecatappl.h"
#include "ecatslv.h"

#include "applInterface.h"
#include "hpm_ecat_hw.h"
#include "hpm_ecat_foe.h"

#include "ecat_foe.h"
#include "cia402appl.h"

extern bool foe_reset_request;

int hpm_ecat_cia402_foe_init(void)
{
    hpm_stat_t stat;
#if !MOTORCONTROL_EC_OR_STUDIO
    board_init_ethercat(HPM_ESC); /* init ESC function pins */
    printf("ECAT FOE Funcation\n");

    /* Config ESC with FOE function to download app */
    stat = ecat_hardware_init(HPM_ESC);
    if (stat != status_success) {
        printf("Init ESC peripheral and related devices(EEPROM/PHY) failed!\n");
        return -1;
    }

     board_delay_ms(1000);
#if defined(CONFIG_CIA402_USING_ACTUAL_MOTOR) && CONFIG_CIA402_USING_ACTUAL_MOTOR
    motor_function_init(); /* motor function init */
#endif
#endif
#if MOTORCONTROL_EC_OR_STUDIO
    motor_function_init();
#endif
#if !MOTORCONTROL_EC_OR_STUDIO
    MainInit(); /* SSC Initialize the stack */

    stat = foe_support_init();
    if (stat != status_success) {
        printf("FOE support init failed!!\n");
        return -1;
    }

    pAPPL_FoeRead = foe_read;
    pAPPL_FoeReadData = foe_read_data;
    pAPPL_FoeWrite = foe_write;
    pAPPL_FoeWriteData = foe_write_data;

#if defined(ESC_EEPROM_EMULATION) && ESC_EEPROM_EMULATION
    pAPPL_EEPROM_Read  = ecat_eeprom_emulation_read;
    pAPPL_EEPROM_Write = ecat_eeprom_emulation_write;
    pAPPL_EEPROM_Reload = ecat_eeprom_emulation_reload;
    pAPPL_EEPROM_Store  = ecat_eeprom_emulation_store;
#endif

    /*Initialize Axes structures*/
    CiA402_Init();
    /* Create basic mapping */
    APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize);
#endif
    /* Set stack run flag */
    bRunApplication = TRUE;  //ethercat、motor初始化成功标记
    /* Execute the stack */
    printf("HPM_PMSM_ECAT_VERSION:   %d", HPM_PMSM_ECAT_VERSION);  //
}

int hpm_ecat_cia402_foe_task(void)
{

    while (bRunApplication == TRUE) 
    {
#if !MOTORCONTROL_EC_OR_STUDIO
        if (foe_reset_request) 
        {
            printf("system reset...\n");
            foe_support_soc_reset();
        }
#endif
#if MOTORCONTROL_EC_OR_STUDIO
       monitor_handle();   
#else
       //monitor_handle();
       MainLoop();
#endif
    }
#if !MOTORCONTROL_EC_OR_STUDIO
      CiA402_DeallocateAxis();
#endif

    /* hardware deinit */

    return 0;
}

