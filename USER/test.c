#include "sys.h"
#include "usart.h"
#include "spi.h"
#include "timer.h"
#include "DIGPOT.h"
#include "GPS.h"
#include "delay.h"
#include "stmflash.h"
#include "24cxx.h"
#include "TMP121.h"


///////////////////////////////////////
////开始全局状态变量定义
///////////////////////////////////////

//定义系统状态结构
SYS_GSV		G_StaV_St ;

u8  		Err_Temp ;							//临时错误标识

SYN_Info	Last_SYN_Table[5]   ;				//最近5次同步信息记录表
u8			Next_SYN_Item	= 0 ;				//指向一个最老的同步或测量信息

u32     SuccTimLast 	= 0 ;        			//最后一次同步时间，记录最后一次同步的本机时间
s32     Correct_m_Last 	= 0 ;     				//最后同步调整值

u32 	PerTimer 		= MS2ON20NS(1000) - 1;	//发射周期，以20ns为单位，默认为1s 50000000-1
u32 	PulW_NAT1P 		= MS2ON20NS(10);   		//第1脉宽，以20ns为单位，默认为10ms

u16 	SuccAdjNum 		= 0  ;          		//成功同步次数

//捕获需要的变量
u8  	AcqTimeResFlag 	= 2 ;                	//捕获测量标志
u32 	AcqTime20ns 	= 0 ;              		//捕获的时延值，以20ns为单位
u32 	AcqTimeS		= 0 ;					//捕获的时延值，以s为单位
u32		AcqGPSTime		= 0 ;					//捕获的GPS时间，也就是说，认为GPS的时间戳是最准确的
//用GPS的脉冲前沿去捕获定时器，捕获到的是本机时间
//如果没有误差，那么AcqTimeS == AcqGPSTime 、AcqTime20ns == 0

//调整变量
u8  	AdjTimeResFlag 	= 2 ;      				//同步标志位
s32 	Correct_m  		= 0;            		//同步调整值

u8  	Timer2_OnOff 	= 1 ;         			//定时器手动打开或关闭,用于调试

u32 	GPS_A_V_TimerOve		= 0 ;			//判断GPS信号是否稳定的标志变量
u32 	GPS_A_V_TimerOve50ms 	= 0 ;

u8		ProgP_SYN_Flag 			= 1 ;			//同步脉冲需要重新同步的标志位
u8		NaviD_Send_Flag 		= 0 ;			//发送本机秒脉冲报文标志
u8		GPSD_Send_Flag			= 0 ;			//发送GPS报文帧标志

//外部脉冲时戳生成功能变量
u8		exPulMeasFlag		= EXPUL_STA_OFF ;	//外部脉冲标志
u32		exPul_TimerOve		= 0 ;				//外部脉冲保护间隔设置
u8 		exPul_TimerOve50ms 	= 0 ;
u32		exPulMeas_ns		= 0 ;				//外部脉冲捕获结果
u32		exPulMeas_s			= 0 ;

//非易失参数
SYS_Para_Save Try_ParaVar ;						//非易失参数临时使用结构

u32 Device_SN0 = 0 ;
u32 Device_SN1 = 0 ;
u32 Device_SN2 = 0 ;

///////////////////////////////////////
////结束全局状态变量定义
///////////////////////////////////////


//////////////////////////////////////////////////
/////设置PPS输出通道 本机pps、GPS pps 或者不输出脉冲
//////////////////////////////////////////////////
u8	PPS_Sel_Flag	= PPS_OFF;

void SetPPSSel(u8 Ch)
{
	if(Ch == PPS_NAV)
	{
		PPS_SEL1 = 0 ;
		PPS_SEL2 = 1 ;
		PPS_Sel_Flag	 = PPS_NAV;
	}
	else if(Ch == PPS_GPS)
	{
		PPS_SEL1 = 1 ;
		PPS_SEL2 = 0 ;
		PPS_Sel_Flag	 = PPS_GPS;
	}
	else
	{
		PPS_SEL1 = 1 ;
		PPS_SEL2 = 1 ;
		PPS_Sel_Flag	 = PPS_OFF;
	}
}
//更新pps输出通道
void refPPSSel(void)
{
	if(G_StaV_St.GPS_STA  == GPS_RNSS_A)	//如果GPS信号有效，则输出GPS的PPS
	{
		if(PPS_Sel_Flag != PPS_GPS)
			SetPPSSel(PPS_GPS);
	}
	else if(G_StaV_St.TKS_Flag == 1)		//如果gps无效，但是本机pps锁定，则输出本机pps
	{
		if(PPS_Sel_Flag != PPS_NAV)
			SetPPSSel(PPS_NAV);
	}
	else									//否则，不输出
	{
		if(PPS_Sel_Flag != PPS_OFF)
			SetPPSSel(PPS_OFF);
	}
}
//////////////////////////////////////////////////
/////初始化PPS输出通道选择
//////////////////////////////////////////////////
void initPPSSel(void)
{
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;														//使能PORTB时钟
	GPIO_Set(GPIOB, PIN1 | PIN2, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); 	//
	SetPPSSel(PPS_OFF);
}

//////////////////////////////////////////////////
/////初始化守时锁定输出
//////////////////////////////////////////////////
void initKeepSta(void)
{
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;														//使能PORTC时钟

	KEEP_STA = 0;
	GPIO_Set(GPIOC, PIN8, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); 		//
}


//////////////////////////////////////
//初始化全局状态结构变量
void InitGlbSt(void)
{
	//获取Flash中的参数
	//Get_Para(&Try_ParaVar);
	AT24CXX_Init();
	Get_Para_IIC(&Try_ParaVar);								//从iic eeprom中获取参数

	//全局变量初始化
	G_StaV_St.RunTime         		= 0;                	//运行时钟
	G_StaV_St.RunTime_50ms  		= 0;                	//50ms为单位的运行时钟

	G_StaV_St.TimeKeepSTA         	= TK_PREOCXO ;         	//主循环状态机
	G_StaV_St.Temper              	= 0;                	//当前芯片温度
	G_StaV_St.DigPot              	= Try_ParaVar.DigPot ;	//数字电位器值，默认为469		*需要存储至flash

	G_StaV_St.TKS_Flag       		= 0 ;               	//守时稳定标识

	G_StaV_St.GPS_STA             	= GPS_RNSS_OFF ;		//GPS状态标示,开机时关闭GPS电源
	G_StaV_St.GPS_Time            	= 0 ;               	//GPS时间
	G_StaV_St.GPS_SatN            	= 0 ;               	//GPS可见星数
	G_StaV_St.GPS_longitude       	= 0.0f;          		//GPS经度
	G_StaV_St.GPS_latitude        	= 0.0f;          		//GPS纬度

	G_StaV_St.NATI_Time  			= 0;					//本机1PPS时戳
	G_StaV_St.NATI_Time_Prog  		= 0;					//本机可编程脉冲时戳

	G_StaV_St.Last_SYN_Time       	= 0 ;					//上次同步时刻
	G_StaV_St.Last_ACQ_s    		= 0 ;					//上次同步测量误差s
	G_StaV_St.Last_ACQ_ns   		= 0 ;					//上次同步测量误差ns
	G_StaV_St.Last_ADJ_s          	= 0 ;					//上次同步调整s
	G_StaV_St.Last_ADJ_ns         	= 0 ;					//上次同步调整s

	G_StaV_St.Auto_SYN				= Try_ParaVar.Auto_SYN ;			//自同步使能					*需要存储至flash
	G_StaV_St.Auto_OCXOADJ			= Try_ParaVar.Auto_OCXOADJ ;		//自调整晶振使能				*需要存储至flash
	G_StaV_St.Prog_Pul				= Try_ParaVar.Prog_Pul ;			//可编程脉冲使能				*需要存储至flash
	G_StaV_St.Auto_KeepSIn			= Try_ParaVar.Auto_KeepSIn ;		//自切换守时状态使能			*需要存储至flash
	G_StaV_St.Meas_Pul				= Try_ParaVar.Meas_Pul ;			//测量功能使能				*需要存储至flash
	G_StaV_St.Prog_PerT				= Try_ParaVar.Prog_PerT; 			//可编程脉冲周期				*需要存储至flash

	G_StaV_St.Messa_SYN				= Try_ParaVar.Messa_SYN ;			//同步信息输出使能			*需要存储至flash
	G_StaV_St.Messa_1PPS			= Try_ParaVar.Messa_1PPS ;			//本机秒脉冲信息使能			*需要存储至flash
	G_StaV_St.Messa_PGP				= Try_ParaVar.Messa_PGP ;			//可编程脉冲信息使能			*需要存储至flash
	G_StaV_St.Messa_GPS				= Try_ParaVar.Messa_GPS ;			//GPS 报文转发使能			*需要存储至flash

	//初始化CRC16表
	getCRCtable2( CRC16, 16, crcTalbe);
}
//////////////////////////////////////
//初始化硬件
void InitHardware(void)
{
	//GPS秒脉冲的门控信号，软件中使用软开关，所以两个门信号均打开
	//GPS秒脉冲门控
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;													//使能PORTC时钟
	GPIO_Set(GPIOA, PIN0, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); 	//
	GATEGPSPPS = 0 ;
	GATEGPSPPS = 1 ;

	//初始化恒温晶振压控数字电位器
	DIGPOT_Init();
	DIGPOT_Write(G_StaV_St.DigPot);

	//初始化GPS时钟，这里为了调试方便默认打开
	GPS_Pow_init();
	GPS_Pow_Re();

	//初始化LED指示灯
	LED_Init();

	//初始化ADC，温度采集
	init_TMP121();

	delay_ms(250);

	//运行时钟初始化
	TIM6_50ms_Init();

	//初始化和主控单片机的串口，这里主要用于测试硬件，不是完整程序,不是完整的协议
	UART6_SYS_init(25, 115200);
	Send_STA_Sys();

	//初始化GPS串口
	UART1_GPS_init(25, 9600);

	//初始化pps通道选择IO
	initPPSSel();
	//初始化守时状态输出IO
	initKeepSta();
}
/////////////////////////////////
////处理每秒钟需要处理的事务
void ProcPer1s(void)
{
	s16		TMP121_Raw		= 0 ;
	float 	Tempr_f			= 0.0f;		//温度滤波器

	//打开或关闭外部脉冲测量功能
	if(G_StaV_St.Meas_Pul == 0)
	{
		if(exPulMeasFlag != EXPUL_STA_OFF)
		{
			exPulMeasFlag = EXPUL_STA_OFF ;
			TIM2_ExPul_Meas_Stop();
		}
	}
	else
	{
		if(exPulMeasFlag == EXPUL_STA_OFF)
		{
			exPulMeasFlag = EXPUL_STA_WAT ;
			TIM2_ExPul_Meas_Start();
		}
	}

	//温度采样
	TMP121_Raw  = Get_TMP_Raw();									//更换为外部的温度传感器
	Tempr_f		= TMP121_Raw * 0.0625f;
	G_StaV_St.Temper	=	(s16)(Tempr_f * 10);					//×10取整，做成S16存储，需要的时候，÷10就是摄氏度

	GPS_Pow_Re();													//刷新GPS电源控制
	refPPSSel();													//刷新PPS通道选择

	//若GPS报文突然没了，把GPS的状态设置为无效
	if(G_StaV_St.GPS_STA == GPS_RNSS_A)								//在GPS有效的状态下，长时间没有收到GPS报文，置为无效
	{
		if( CheckTimerOve(GPS_A_V_TimerOve, GPS_A_V_TimerOve50ms) == 1)
			G_StaV_St.GPS_STA = GPS_RNSS_V ;
	}
}

//////////////////////////////////////
//主函数
int main(void)
{
	u8  Temp_AcqTimeRes ;				//用于缓冲采集时序标志位，担心状态锁死
	u8  Temp_AdjTimeRes ;				//用于缓冲调整时序标志位，担心状态锁死

	u8 	New_CMD_Flag = 0;				//新的串口命令，来自上位机
	u8 	New_Data[301];					//新的串口命令报文，数据域，来自上位机

	u32  SYN_TimerOveSec 	= 0; 		//同步测试使用的超时变量
	u8   SYN_TimerOve50ms	= 0;

	u32  Last_SYN_Time_ed	= 0;		//上一次用于判断是否锁定的时戳

	u8		Per1s_Flag 		= 0;		//每1s干一次的标志变量

	double	Syn_Err 		= 0.0f;		//同步误差变量

	delay_init(10);						//这里的延时一定要加，等待晶振稳定，恒温晶振初始的几百ms需要稳定一下
	delay_ms(500);
	delay_ms(500);

	//内部RC，50MHz，测试使用
	//Stm32_Clock_Init_HSI(100, 8, 4, 7); 	//设置时钟,50Mhz
	//APB1 25MHz TIM2/3/4/5/6/7 50MHz
	//APB2 25MHz TIM1/8/9/10/11 50MHz

	//恒温晶振，50MHz
	Stm32_Clock_Init(100, 10, 2, 7);   		//设置时钟,50Mhz
	//APB1 25MHz TIM2/3/4/5/6/7 50MHz
	//APB2 25MHz TIM1/8/9/10/11 50MHz

	delay_init(50);                     	//延时初始化
	delay_ms(250); 							//FLASH操作之前需要延时

	InitGlbSt();							//初始化结构变量
	InitHardware();							//初始化硬件

	//初始化主计数器TIM2
	TIM2_KEEP_Init(PerTimer, PulW_NAT1P);				//初始化定时器2，主定时器，这个定时器维护着本机1s脉冲，NAT1S
	//模块的所有功能基本都来自这个定时器
	//CH3->本机秒脉冲
	//CH2->捕获GPS秒脉冲
	//CH1->用于半周期调整中断
	//CH4->待测外部脉冲输入端口

	delay_ms(250);   									//在进入主循环之前，再演示250ms，稳定一下

	SYSSTALED	=	LED_Flash_100Ms;					//预热状态
	TKPSTALED	= 	LED_Flash_100Ms;

	//主循环开始，再也不会停下来
	while(1)											//主循环从此处开始
	{
		//每秒更新一次,主要是ADC电压采集转成其他物理量，后面可能还要增加
		if( G_StaV_St.RunTime_50ms >= 9 && Per1s_Flag == 0 )
		{
			Per1s_Flag = 1;														//确保，每一秒，只会进入一次
			ProcPer1s();														//每秒钟需要处理的事务
		}

		//每秒处理结束！

		//处理 外部脉冲测量功能 事务
		if(G_StaV_St.Meas_Pul == 1)
		{
			if(exPulMeasFlag == EXPUL_STA_IED)
			{
				Send_Sys_MeasPlu(exPulMeas_s, exPulMeas_ns);
				exPulMeasFlag = EXPUL_STA_PRT ;									//间隔保护状态
			}
			else if(exPulMeasFlag == EXPUL_STA_PRT)
			{
				if( CheckTimerOve(exPul_TimerOve, exPul_TimerOve50ms) == 1)		//间隔保护结束
				{
					exPulMeasFlag = EXPUL_STA_WAT ;								//重新打开外部脉冲捕获
					TIM2_ExPul_Meas_Start();
				}
			}
		}

		//处理 PPS信息转发功能 事务
		if(G_StaV_St.Messa_1PPS == 1)
		{
			//如果允许秒脉冲报文发射，并且有新的秒脉冲产生，且已经锁定或GPS有效，则发射秒脉冲报文
			if(	PPS_Sel_Flag == PPS_NAV	&& NaviD_Send_Flag == 1)
			{
				Send_Sys_NavPPS();
				NaviD_Send_Flag = 0 ;
			}
			else if( PPS_Sel_Flag == PPS_GPS && GPSD_Send_Flag == 1)
			{
				Send_Sys_GPSPPS();
				GPSD_Send_Flag = 0 ;
			}
			else			//GPS无效，并且没有锁定
			{
				NaviD_Send_Flag = 0 ;
				GPSD_Send_Flag 	= 0 ;
			}
		}
		else				//如果关闭pps脉冲信息  什么都不发送
		{
			NaviD_Send_Flag = 0 ;
			GPSD_Send_Flag 	= 0 ;
		}

		//GPS报文顶层处理
		Prorc_GPS_Top();

		//处理原始的接收缓冲区未处理字节，解析出有效接收帧
		Proc_RxByte_SYS();
		//取出最老的一个接收有效帧
		New_CMD_Flag = Proc_RxCMD_SYS(New_Data);
		New_CMD_Flag = ProcPC_CMD(New_CMD_Flag, New_Data);		//处理常见报文事务
		//‘S’切换状态在各个状态里处理
		//处理串口发送事务,如果有有效帧需要发送且发送DMA没有在工作,
		//那么把最老的一帧使用DMA发送出去.否则直接退出.
		Proc_TxFrame_SYS();

		////////////////////////////////////////////////////////////////////////////
		/////////
		/////////    主循环的状态机从此开始
		/////////
		////////////////////////////////////////////////////////////////////////////
		switch(G_StaV_St.TimeKeepSTA)
		{
			//预热状态
			case TK_PREOCXO:

				if(G_StaV_St.RunTime > 120)
				{
					SYSSTALED	=	LED_Flash_1s;
					G_StaV_St.TimeKeepSTA = TK_IDLE;
					Send_STA_Sys();
				}

				if(New_CMD_Flag == 'S')							//手动进入同步状态
				{
					if(New_Data[0] == 'S')
					{
						SYSSTALED	=	LED_Flash_1s;
						G_StaV_St.TimeKeepSTA = TK_IDLE;
						Send_STA_Sys();
					}
				}

				break;

			//空闲状态，又称同步状态
			case TK_IDLE:                                  		//空闲状态***文档中称为“同步状态”

				//自同步，满足，同步或测量条件时，开始一次测量或者同步操作
				if( G_StaV_St.Auto_SYN == 1 && G_StaV_St.GPS_STA == GPS_RNSS_A )
				{
					//这里启动一次  同步并测量
					if( (	G_StaV_St.GPS_Time % 86400) % (1200 / 10) == (1200 - 10) / 10 				//满足同步条件的秒脉冲即将发生,2min(60s)的整倍数，的前一秒
							&& G_StaV_St.GPS_Time >= (G_StaV_St.Last_SYN_Time + (1200 - 10) / 10) 		//当前的GPS时间比上次同步时间多一个同步周期少1s,防止多次进入同步
							&& CheckTimerOve(SYN_TimerOveSec, SYN_TimerOve50ms) == 1 					//上次同步结束2s以上,才可进入同步
					  )
					{
						AcqGPSTime		= G_StaV_St.GPS_Time + 1;	//下一个GPS秒脉冲的时间戳，正常情况，这下一个秒脉冲的前沿就是我们要用来同步的脉冲
						AcqTimeResFlag 	= 0 ;						//初始化测量标志位
						TIM2_Meas_Start();              			//启动测量序列
						G_StaV_St.TimeKeepSTA = TK_RUNSYN_MEAS ;  	//进入测量并同步中的测量状态,测量段
					}
				}
				else if( G_StaV_St.Auto_SYN == 0 && G_StaV_St.GPS_STA == GPS_RNSS_A )					//只测量，不同步
				{
					//这里启动一次  单测量
					if( (	G_StaV_St.GPS_Time % 86400) % (1200 / 10) == (1200 - 10) / 10 				//满足同步条件的秒脉冲即将发生,2min(60s)的整倍数，的前一秒
							&& G_StaV_St.GPS_Time >= (G_StaV_St.Last_SYN_Time + (1200 - 10) / 10) 		//当前的GPS时间比上次同步时间多一个同步周期少1s,防止多次进入同步
							&& CheckTimerOve(SYN_TimerOveSec, SYN_TimerOve50ms) == 1 					//上次同步结束2s以上,才可进入同步
					  )
					{
						AcqGPSTime		= G_StaV_St.GPS_Time + 1;									//下一个GPS秒脉冲的时间戳，正常情况，这下一个秒脉冲的前沿就是我们要用来测量的脉冲
						AcqTimeResFlag 	= 0 ;            											//初始化测量标志位
						TIM2_Meas_Start();              											//启动测量序列
						G_StaV_St.TimeKeepSTA = TK_RUNMEAS ;      									//进入单测量同步脉冲相对时延状态
					}
				}

				//根据最近一次的同步结果，判断是否完成锁定
				if( 	Last_SYN_Time_ed != G_StaV_St.Last_SYN_Time 		//有新的同步点，可用于判断是否锁定
						&& 	G_StaV_St.RunTime > 900 						//必须上电 900s 才能判断是否锁定，考虑恒温晶振
						&& 	G_StaV_St.Last_SYN_Time != 0)					//如果没有同步1次，不进行判断
				{
					Last_SYN_Time_ed = G_StaV_St.Last_SYN_Time ;

					Syn_Err = (double)G_StaV_St.Last_SYN_Time - (double)G_StaV_St.Last_ACQ_s ;

					Syn_Err = Syn_Err - ((double)G_StaV_St.Last_ACQ_ns) * 20.0f / 1000000000.0f ;

					if(Syn_Err < 0)
						Syn_Err = -Syn_Err ;

					if(Syn_Err < 0.000001)					//如果小于1us，认为锁定
					{
						TKPSTALED	= 	LED_Flash_2s;

						G_StaV_St.TKS_Flag 	= 1 ;
						KEEP_STA 			= 1;
					}
					else
					{
						TKPSTALED	= 	LED_Flash_100Ms;	//没有锁定

						G_StaV_St.TKS_Flag 	= 0 ;
						KEEP_STA 			= 0;
					}
				}

				if(G_StaV_St.TimeKeepSTA == TK_IDLE)		//上面有可能切换状态，所以，先保证，前面没有进入其他同步或者测量子状态
				{
					if(G_StaV_St.Auto_KeepSIn == 1 && G_StaV_St.TKS_Flag == 1 )	//自动切换至守时状态
					{
						SYSSTALED	=	LED_Flash_2s;
						G_StaV_St.TimeKeepSTA = TK_TIMKEEP ;
						Send_STA_Sys();
					}

					if(New_CMD_Flag == 'S')										//手动进入守时状态
					{
						if(New_Data[0] == 'K')
						{
							SYSSTALED	=	LED_Flash_2s;
							G_StaV_St.TimeKeepSTA = TK_TIMKEEP ;
							Send_STA_Sys();
						}
					}
				}

				break;

			//守时状态，此状态下不同步，GPS关闭
			case TK_TIMKEEP:

				if(New_CMD_Flag == 'S')											//手动进入同步状态
				{
					if(New_Data[0] == 'S')
					{
						G_StaV_St.TKS_Flag 		= 0 ;
						KEEP_STA 				= 0	;

						TKPSTALED				= LED_Flash_100Ms;
						G_StaV_St.Last_ACQ_s 	= 0 ;
						G_StaV_St.Last_ACQ_ns   = 0 ;

						SYSSTALED				= LED_Flash_1s;
						G_StaV_St.TimeKeepSTA 	= TK_IDLE;
						Send_STA_Sys();
					}
				}

				break;

			case TK_RUNMEAS:                        		//单测量运行状态***

				//在这个状态下，等到测量到时延或者超时
				Temp_AcqTimeRes = AcqTimeResFlag ;    		//取出变量,防止在if语句中出现变量被中断修改的错误

				if( Temp_AcqTimeRes == 1 )            		//测量成功
				{
					G_StaV_St.TimeKeepSTA = TK_COMMEAS ;
				}
				else if( Temp_AcqTimeRes == 2 )       		//测量超时
				{
					G_StaV_St.TimeKeepSTA = TK_OVEMEAS ;
				}
				else if( Temp_AcqTimeRes == 0 )       		//测量进行中
					G_StaV_St.TimeKeepSTA = TK_RUNMEAS ;
				else
				{
					//异常，报错，退回空闲
					Err_Temp = 2;
					Send_Err('M', &Err_Temp, 1);
					G_StaV_St.TimeKeepSTA = TK_IDLE ;
				}

				break;

			case TK_COMMEAS:                        		//单测量完成状态***

				G_StaV_St.Last_SYN_Time						=	AcqGPSTime ;		//上次同步时刻
				G_StaV_St.Last_ACQ_s						=	AcqTimeS ;			//上次同步测量s
				G_StaV_St.Last_ACQ_ns						=	AcqTime20ns ;		//上次同步测量ns
				G_StaV_St.Last_ADJ_s						=	0 ;					//上次同步调整s,没有调整，只是测量，这里写0
				G_StaV_St.Last_ADJ_ns						=	0 ;					//上次同步调整s

				//保存到最近5次同步信息数组结构
				Last_SYN_Table[Next_SYN_Item].SYN_Time		=	AcqGPSTime ;		//上次同步时刻
				Last_SYN_Table[Next_SYN_Item].ACQ_s			=	AcqTimeS ;			//上次同步测量捕获s
				Last_SYN_Table[Next_SYN_Item].ACQ_ns		=	AcqTime20ns ;		//上次同步测量捕获ns
				Last_SYN_Table[Next_SYN_Item].ADJ_s			=	0 ;					//上次同步调整s
				Last_SYN_Table[Next_SYN_Item].ADJ_ns		=	0 ;					//上次同步调整s
				Next_SYN_Item++;
				Next_SYN_Item %= 5;

				SYN_TimerOve50ms	= G_StaV_St.RunTime_50ms ;    					//收到结果后2秒内不再进入同步过程
				SYN_TimerOveSec 	= G_StaV_St.RunTime + 2;                		//主要是防止,同步调整很快就结束,错误进入第二次同步序列

				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				//上报结果
				if(G_StaV_St.Messa_SYN == 1)
				{
					Send_Meas_Adj_Res();
				}

				break;

			case TK_OVEMEAS:                        	//单测量超时状态***

				//上报错误超时
				Err_Temp = 1;
				Send_Err('M', &Err_Temp, 1);
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;

			case TK_RUNSYN_MEAS:                    	//同步测量段运行状态***

				//在这个状态下，等到测量到时延或者超时
				Temp_AcqTimeRes = AcqTimeResFlag ;    	//取出变量,防止在if语句中出现变量被中断修改的错误

				if( Temp_AcqTimeRes == 1)
				{
					G_StaV_St.TimeKeepSTA = TK_COMSYN_MEAS ;
				}
				else if( Temp_AcqTimeRes == 2)
				{
					G_StaV_St.TimeKeepSTA = TK_OVESYN_MEAS ;
				}
				else if( Temp_AcqTimeRes == 0 )
					G_StaV_St.TimeKeepSTA = TK_RUNSYN_MEAS ;
				else
				{
					//异常，报错，退回空闲
					Err_Temp = 2;
					Send_Err('S', &Err_Temp, 1);
					G_StaV_St.TimeKeepSTA = TK_IDLE ;
				}

				break;

			case TK_COMSYN_MEAS:                    					//同步测量段完成状态***

				//根据测量结果，计算调整数值
				Correct_m = AdjCalc(PerTimer, AcqTime20ns, PulW_NAT1P);	//根据捕获值计算调整量

				//打开调整序列程序
				AdjTimeResFlag = 0 ;
				TIM2_Adj_Start();

				G_StaV_St.TimeKeepSTA = TK_RUNSYN_ADJ ;           		//进入调整状态

				break;

			case TK_OVESYN_MEAS:                        				//同步测量段超时状态***
				//上报错误超时
				Err_Temp = 1;
				Send_Err('S', &Err_Temp, 1);							//测量段超时
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;

			case TK_RUNSYN_ADJ:                         				//同步调整段运行状态***

				//在这个状态下，等到调整结束或者超时
				Temp_AdjTimeRes = AdjTimeResFlag ;      				//取出变量,防止在if语句中出现变量被中断修改的错误

				if( Temp_AdjTimeRes == 1)//调整结束
				{
					G_StaV_St.TimeKeepSTA = TK_COMSYN_ADJ ;
				}
				else if( Temp_AdjTimeRes == 2)
				{
					G_StaV_St.TimeKeepSTA = TK_OVESYN_ADJ ;
				}
				else if( Temp_AdjTimeRes == 0)
					G_StaV_St.TimeKeepSTA = TK_RUNSYN_ADJ ;
				else
				{
					//异常，报错，退回空闲
					Err_Temp = 3;
					Send_Err('S', &Err_Temp, 1);						//调整段异常,这个地方异常的可能性比较小
					G_StaV_St.TimeKeepSTA = TK_IDLE ;
				}

				break;

			case TK_COMSYN_ADJ:											//同步调整段成功状态***

				G_StaV_St.Last_SYN_Time						=	AcqGPSTime ;		//上次同步时刻
				G_StaV_St.Last_ACQ_s						=	AcqTimeS ;			//上次同步测量s
				G_StaV_St.Last_ACQ_ns						=	AcqTime20ns ;		//上次同步测量ns
				G_StaV_St.Last_ADJ_s						=	AcqGPSTime ;		//上次同步调整s，这里是直接设置成GPS的时间，为了简单和与之前代码兼容
				G_StaV_St.Last_ADJ_ns						=	Correct_m ;			//上次同步调整s

				//保存到最近5次同步信息数组结构
				Last_SYN_Table[Next_SYN_Item].SYN_Time		=	AcqGPSTime ;		//上次同步时刻
				Last_SYN_Table[Next_SYN_Item].ACQ_s			=	AcqTimeS ;			//上次同步测量捕获s
				Last_SYN_Table[Next_SYN_Item].ACQ_ns		=	AcqTime20ns ;		//上次同步测量捕获ns
				Last_SYN_Table[Next_SYN_Item].ADJ_s			=	AcqGPSTime;			//上次同步调整s
				Last_SYN_Table[Next_SYN_Item].ADJ_ns		=	Correct_m;			//上次同步调整s
				Next_SYN_Item++;
				Next_SYN_Item %= 5;

				SYN_TimerOve50ms	= G_StaV_St.RunTime_50ms ;    					//收到结果后2秒内不再进入同步过程
				SYN_TimerOveSec 	= G_StaV_St.RunTime + 2;						//主要是防止,同步调整很快就结束,错误进入第二次同步序列

				SuccAdjNum ++  ;

				G_StaV_St.TimeKeepSTA		= 	TK_IDLE ;

				//上报结果，包括测量和调整数值
				if(G_StaV_St.Messa_SYN == 1)
				{
					Send_Meas_Adj_Res();
				}

				break;

			case TK_OVESYN_ADJ:                         //同步调整段超时状态***
				//上报错误超时
				Err_Temp = 4;
				Send_Err('S', &Err_Temp, 1);            //调整段超时
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;

			default:                                    //未定义的状态***

				//上报未知状态错误
				Err_Temp = 1;
				Send_Err('Q', &Err_Temp, 1);            //未知状态错误
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;
		}

		//更新1s标志变量，在这里清零1次后，这一秒不会再进入了
		if( G_StaV_St.RunTime_50ms == 0 && ( Per1s_Flag == 1 ) )
		{
			Per1s_Flag = 0 ;
		}
	}
}
//

