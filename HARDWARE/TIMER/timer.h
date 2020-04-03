#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

u16 CheckTimerOve(u32 Sec, u8 Ms200);

void Pul2nd_Enable(u8 En);

void TIM3_Int_Init(u16 arr, u16 psc);
void TIM14_PWM_Init(u32 arr, u32 psc);
void TIM5_CH1_Cap_Init(u32 arr, u16 psc);
void TIM9_CH2_PWM_Init(u16 arr, u16 psc);

void TIM2_CH3_1PPS_Init(void);
void TIM3_2nd_Test(void);

void TIM2_ONOFF( u8 OnOff );
void TIM2_CH2_GPS1PPS_Meas(void);
void TIM2_CH1_GPS1PPS_Adj(void);

void TIM6_50ms_Init(void);
void TIM2_KEEP_Init(u32 PerTimer, u32 PulW_NAT1P); //,u8 SYN_CH);
void TIM2_Meas_Start(void);
void TIM2_Adj_Start(void);
void TIM2_CH3_Test_Init(void);
void TIM3_2nd_Init(u16 Inter_ZeroI, u16 Inter_MaxI, u16 Inter_TI, u32 PulW_2ndI );
void Pul2nd_Delay_Set(u16 Inter_TI) ;
void Set_PulW_NAT1P(u32 PulW_NAT1PI);
s32 AdjCalc(u32 PerT, u32 AcqD, u32 PulW_NAT1P);
void TIM2_ExPul_Meas_Start(void);
void TIM2_ExPul_Meas_Stop(void);
#endif























