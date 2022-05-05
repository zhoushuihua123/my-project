/******************************************************************************	
* File Name    : pwm_init
* Version      : 1.00
* Device    : RX62T
* Tool-Chain   : RX 1.0.0.0
* H/W Platform : RX62TRSK
* Description  : this file shows pwm initialize procedure
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 07.06.2011 1.00    First Release
******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
//#include <machine.h>
#include	"incfile.h"

#include "spc1158.h"
#include <stdio.h>
#include "motor_pwm.h" 
#include "motor_sys_config_hardware.h"

#include "ConstantDef.h"
 
 zhoushuihua 
 

// PWM和AD采样初始化
void CompPWM_ADSetInit(void)
{
     UI_32 fui32CarrierHz;
     UI_32 fui32MtrDeadtime;
 
    fui32CarrierHz = (UI_32)gdef_ui16MTR_FC_NUM_T;        // 载波频率 
    fui32MtrDeadtime = (UI_32)ucDrvEepPara[30] * 25;  		// 死区时间(单位ns),ucDrvEepPara[30]是乘以了40，再乘以25，表示单位是ns。

//    dpinv0_init(fui32Carrier, fui16MtrDeadtime, 1); 

    /*----------------------------------------------------------------- 
       Step 1: Initial Basic Complementary PWM
  ------------------------------------------------------------------*/
    PWM_ComplementaryPairChannelInit(PWM_U , fui32CarrierHz/* PWM frequency (Hz) */, fui32MtrDeadtime /* Deadtime(ns) */);    
    PWM_ComplementaryPairChannelInit(PWM_V , fui32CarrierHz/* PWM frequency (Hz) */, fui32MtrDeadtime /* Deadtime(ns) */);    
    PWM_ComplementaryPairChannelInit(PWM_W , fui32CarrierHz/* PWM frequency (Hz) */, fui32MtrDeadtime /* Deadtime(ns) */);    
 
    /* For SVPWM and PWM set duty*/
    gu32PWMPeriod = PWM_U->TBPRD.bit.VAL;			// gu32PWMPeriod是PWM波峰计数值
		_DPINV0_PERIOD = gu32PWMPeriod;
		
	  gui16MTRbootOutput = _DPINV0_PERIOD >> 5;
    gui16MTRbootOutput2 = _DPINV0_PERIOD >> 2;
	
    /*  Sycn Config */
    PWM_U->TBCTL.all |= TBCTL_ALL_SYNCOSEL_TBCNT_EQU_ZERO;
    PWM_V->TBCTL.all |= TBCTL_ALL_PHSEN_ENABLE
                     |  TBCTL_ALL_PHSDIR_COUNT_UP_AFTER_SYNC;
    PWM_W->TBCTL.all |= TBCTL_ALL_PHSEN_ENABLE
                     |  TBCTL_ALL_PHSDIR_COUNT_UP_AFTER_SYNC;	


    /*  Step 5: Set PWM output waveform, central alignment and CMPA with duty are propotional  */
    PWM_U->AQCTLA.all = AQCTLA_ALL_ZRO_SET_LOW		/* Do not change output while counter reaches zero  */		
                       |AQCTLA_ALL_CAU_SET_HIGH      
                       |AQCTLA_ALL_CAD_SET_LOW;     

    PWM_V->AQCTLA.all =  AQCTLA_ALL_ZRO_SET_LOW		/* Do not change output while counter reaches zero  */		
                        |AQCTLA_ALL_CAU_SET_HIGH      
                        |AQCTLA_ALL_CAD_SET_LOW;


    PWM_W->AQCTLA.all =  AQCTLA_ALL_ZRO_SET_LOW		/* Do not change output while counter reaches zero  */		
                        |AQCTLA_ALL_CAU_SET_HIGH      
                        |AQCTLA_ALL_CAD_SET_LOW;
		
		  /* Step 6: Waveform generating with Dead-time */
#if IPM_IGBT_OutPutLogic == 0	// 分立IGBT+士兰驱动HVICSDH2136方案，驱动信号相反  		  
  PWM_U->DBCTL.all  = 0;
  PWM_U->DBCTL.all |=   DBCTL_ALL_FEDSRC_FROM_A 
                     | DBCTL_ALL_REDSRC_FROM_A 
                     | DBCTL_ALL_OUTASRC_RISING_EDGE 
                     | DBCTL_ALL_OUTBSRC_FALLING_EDGE 
                     | DBCTL_ALL_REDPOL_ACTIVE_LOW 
                     | DBCTL_ALL_FEDPOL_ACTIVE_HIGH  
                     | DBCTL_ALL_HALFCYCLE_DISABLE 
                     | DBCTL_ALL_REDEN_ENABLE 
                     | DBCTL_ALL_FEDEN_ENABLE;
										 
	PWM_V->DBCTL.all  = 0;
  PWM_V->DBCTL.all |=   DBCTL_ALL_FEDSRC_FROM_A 
                     | DBCTL_ALL_REDSRC_FROM_A 
                     | DBCTL_ALL_OUTASRC_RISING_EDGE 
                     | DBCTL_ALL_OUTBSRC_FALLING_EDGE 
                     | DBCTL_ALL_REDPOL_ACTIVE_LOW 
                     | DBCTL_ALL_FEDPOL_ACTIVE_HIGH  
                     | DBCTL_ALL_HALFCYCLE_DISABLE 
                     | DBCTL_ALL_REDEN_ENABLE 
                     | DBCTL_ALL_FEDEN_ENABLE;		
										 
	PWM_W->DBCTL.all  = 0;
  PWM_W->DBCTL.all |=   DBCTL_ALL_FEDSRC_FROM_A 
                     | DBCTL_ALL_REDSRC_FROM_A 
                     | DBCTL_ALL_OUTASRC_RISING_EDGE 
                     | DBCTL_ALL_OUTBSRC_FALLING_EDGE 
                     | DBCTL_ALL_REDPOL_ACTIVE_LOW 
                     | DBCTL_ALL_FEDPOL_ACTIVE_HIGH  
                     | DBCTL_ALL_HALFCYCLE_DISABLE 
                     | DBCTL_ALL_REDEN_ENABLE 
                     | DBCTL_ALL_FEDEN_ENABLE;									 
#else
  PWM_U->DBCTL.all  = 0;
  PWM_U->DBCTL.all |=   DBCTL_ALL_FEDSRC_FROM_A 
                     | DBCTL_ALL_REDSRC_FROM_A 
                     | DBCTL_ALL_OUTASRC_RISING_EDGE 
                     | DBCTL_ALL_OUTBSRC_FALLING_EDGE 
                     | DBCTL_ALL_REDPOL_ACTIVE_HIGH 
                     | DBCTL_ALL_FEDPOL_ACTIVE_LOW 
                     | DBCTL_ALL_HALFCYCLE_DISABLE 
                     | DBCTL_ALL_REDEN_ENABLE 
                     | DBCTL_ALL_FEDEN_ENABLE; 
										 
  PWM_V->DBCTL.all  = 0;
  PWM_V->DBCTL.all |=   DBCTL_ALL_FEDSRC_FROM_A 
                     | DBCTL_ALL_REDSRC_FROM_A 
                     | DBCTL_ALL_OUTASRC_RISING_EDGE 
                     | DBCTL_ALL_OUTBSRC_FALLING_EDGE 
                     | DBCTL_ALL_REDPOL_ACTIVE_HIGH 
                     | DBCTL_ALL_FEDPOL_ACTIVE_LOW 
                     | DBCTL_ALL_HALFCYCLE_DISABLE 
                     | DBCTL_ALL_REDEN_ENABLE 
                     | DBCTL_ALL_FEDEN_ENABLE; 				

  PWM_W->DBCTL.all  = 0;
  PWM_W->DBCTL.all |=   DBCTL_ALL_FEDSRC_FROM_A 
                     | DBCTL_ALL_REDSRC_FROM_A 
                     | DBCTL_ALL_OUTASRC_RISING_EDGE 
                     | DBCTL_ALL_OUTBSRC_FALLING_EDGE 
                     | DBCTL_ALL_REDPOL_ACTIVE_HIGH 
                     | DBCTL_ALL_FEDPOL_ACTIVE_LOW 
                     | DBCTL_ALL_HALFCYCLE_DISABLE 
                     | DBCTL_ALL_REDEN_ENABLE 
                     | DBCTL_ALL_FEDEN_ENABLE; 
#endif                     

		// 硬件是反的，H对应OUTB，L对应OUTA
    PWM_U->DBCTL.bit.OUTASRC = 0;
    PWM_U->DBCTL.bit.OUTBSRC = 1;
    PWM_V->DBCTL.bit.OUTASRC = 0;
    PWM_V->DBCTL.bit.OUTBSRC = 1;
    PWM_W->DBCTL.bit.OUTASRC = 0;
    PWM_W->DBCTL.bit.OUTBSRC = 1;

/* CMP AD setting */
//	dpinv0_1shunt_ad0_init();  //AD0's initial and AD conversion's start configuration

    // 设置PWM中断源(V相波峰和U相波谷分别触发)和AD触发源(U相SOCA、SOCC，V相SOCC)
    MotorPWM_OneShuntTriggerADCInit();
    
    //AD初始化，设置AD电流采样触发时间和直流母线电压采样触发时间
    MotorADC_ADCandPGAInit();
    SetCompAD_Start(_DPINV0_PERIOD>>1, _DPINV0_PERIOD >> 2); 
  
  	// 设置内部比较器
#if ChooseMCUCompare == 1					// 当需要内部比较器过流保护时
    	MotorPWM_InitTripZoneAction();
#endif
  
   	// 设置50%占空比，PWM引脚不输出
   	DisableCompPWM();
  	MotorPWM_SetUVWDuty_Zero();
    // 开启PWM时钟，启动PWM
    MotorPWM_StartUVWClock(PWM_U,PWM_V,PWM_W);
  
  	// 开启PWM中断
		NVIC_ClearPendingIRQ(PWM_V_PeakIRQn);   // 波峰中断
    NVIC_SetPriority(PWM_V_PeakIRQn,2);
    NVIC_EnableIRQ(PWM_V_PeakIRQn);
    PWM_ClearTimeEventInt(PWM_V);
    
    NVIC_ClearPendingIRQ(PWM_U_ButtomIRQn);   // 波谷中断
    NVIC_SetPriority(PWM_U_ButtomIRQn,2);
    NVIC_EnableIRQ(PWM_U_ButtomIRQn);
    PWM_ClearTimeEventInt(PWM_U);
     
}
 
 
 

  
