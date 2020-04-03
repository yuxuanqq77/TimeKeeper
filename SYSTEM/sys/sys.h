#ifndef __SYS_H
#define __SYS_H
#include "stm32f4xx.h"

#include <rtl.h>

//crc16 多项式掩码
#define 	CRC16		0x8408			//CRC-CCITT:x16+x12+x5+1  0x8408

#define Soft_Ver 	210
#define Hard_Ver 	110

#define KEEP_STA    PCout(8) 			//守时锁定输出IO

#define PPS_NAV		1
#define PPS_GPS		2
#define PPS_OFF		0

#define EXPUL_STA_OFF		0
#define EXPUL_STA_WAT		1
#define EXPUL_STA_IED		2
#define EXPUL_STA_PRT		3

#define ADC_CH_TEMP  	16 		 	  	//通道16,内部温度传感器专用通道	   	  

#define FLASH_SAVE_ADDR  0X080E0000

//////////////////////////////////////////////////////////////////////////
#define TK_PREOCXO          0       		//预热状态***

#define TK_IDLE             1             	//空闲状态*** 同步状态

#define TK_RUNMEAS          2             	//单测量运行状态***
#define TK_COMMEAS          3             	//单测量成功状态***
#define TK_OVEMEAS          4             	//单测量超时状态***

#define TK_RUNSYN_MEAS      5             	//同步测量段运行状态***
#define TK_COMSYN_MEAS      6             	//同步测量段成功状态***
#define TK_OVESYN_MEAS      7             	//同步测量段超时状态***

#define TK_RUNSYN_ADJ       8             	//同步调整段运行状态***
#define TK_COMSYN_ADJ       9             	//同步调整段完成状态***
#define TK_OVESYN_ADJ       10				//同步调整段超时状态***

#define TK_TIMKEEP          11   			//守时状态***
//////////////////////////////////////////////////////////////////////////

__packed typedef struct
{
	u8 Sec;
	u8 Min;
	u8 Hor;
	u8 Day;
	u8 Dat;
	u8 Mon;
	u8 Yea;

	u8 BCD_Sec;
	u8 BCD_Min;
	u8 BCD_Hor;
	u8 BCD_Day;
	u8 BCD_Dat;
	u8 BCD_Mon;
	u8 BCD_Yea;

	u16 QYear;
	u32 Unix_Time;
} RTC_TIME_Str;

__packed typedef struct
{
	u32     SYN_Time;			//上次同步时刻
	u32    	ACQ_s;				//上次同步测量捕获s
	u32    	ACQ_ns;				//上次同步测量捕获ns
	u32    	ADJ_s;				//上次同步调整s
	s32    	ADJ_ns;				//上次同步调整s
} SYN_Info;


//定义系统状态变量结构
typedef struct
{
	u32     RunTime;                //运行时钟
	u8      RunTime_50ms;   		//50ms为单位的运行时钟

	u8      TimeKeepSTA ;         	//主循环状态机
	s16     Temper;                	//当前芯片温度
	u16     DigPot;             	//数字电位器值，默认为469

	u8      TKS_Flag;        		//守时稳定标识

	u8      GPS_STA;         		//GPS状态标示
	u32     GPS_Time;               //GPS时间
	u8      GPS_SatN;               //GPS可见星数
	double  GPS_longitude;     		//GPS经度
	double  GPS_latitude;      		//GPS纬度

	u32     NATI_Time;				//本机1PPS时戳
	u32     NATI_Time_Prog;			//本机可编程脉冲时戳

	u32     Last_SYN_Time;			//上次同步时刻
	u32     Last_ACQ_s;				//上次同步测量捕获s
	u32     Last_ACQ_ns;			//上次同步测量捕获ns
	u32     Last_ADJ_s;				//上次同步调整s
	s32     Last_ADJ_ns;			//上次同步调整s

	u8		Auto_SYN;				//自同步使能
	u8		Auto_OCXOADJ;			//自调整晶振使能
	u8		Prog_Pul;				//可编程脉冲使能
	u8		Auto_KeepSIn;			//自切换守时状态使能
	u8		Meas_Pul;				//测量功能使能
	u32		Prog_PerT;				//可编程脉冲周期


	u8		Messa_SYN;				//同步信息输出使能
	u8		Messa_1PPS;				//本机秒脉冲信息使能
	u8		Messa_PGP;				//可编程脉冲信息使能
	u8		Messa_GPS;				//GPS 报文转发使能
} SYS_GSV ;

typedef struct
{
	u32     DigPot;             	//数字电位器值，默认为469
	u32		Auto_SYN;				//自同步使能
	u32		Auto_OCXOADJ;			//自调整晶振使能
	u32		Prog_Pul;				//可编程脉冲使能
	u32		Auto_KeepSIn;			//自切换守时状态使能
	u32		Meas_Pul;				//测量功能使能
	u32		Prog_PerT;				//可编程脉冲周期
	u32		Messa_SYN;				//同步信息输出使能
	u32		Messa_1PPS;				//本机秒脉冲信息使能
	u32		Messa_PGP;				//可编程脉冲信息使能
	u32		Messa_GPS;				//GPS 报文转发使能
} SYS_Para_Save ;


#define GPS_RNSS_OFF  0
#define GPS_RNSS_V    1
#define GPS_RNSS_A    2


//V1.1 加入GPS RMC GSV两种报文转发功能至主控单片机
//Timer.c中 Adj_OVETime = (PerTimer+1)/10/1000/1000 + 1 ;
//改为Adj_OVETime = (PerTimer+1)/10/1000/1000 + 2 ;防止100ms周期时,一设置就超时的错误

#define MS2ON20NS(x) x*1000000/20
#define MS2ON5US(x)  x*1000/5

#define GATEGPSPPS  PAout(0)	// 

#define LED1        PAout(12)	// 
#define LED2 		PAout(11)	//

#define POWGPS      PCout(5)	// 
#define GPS_P_C   	PCin(9)  	//GPS外控引脚 

#define PPS_SEL1  	PBout(2)	// 
#define PPS_SEL2  	PBout(1)	// 

//0,不支持ucos
//1,支持ucos
#define SYSTEM_SUPPORT_UCOS		0		//定义系统文件夹是否支持UCOS


//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入
//////////////////////////////////////////////////////////////////////////////////
//Ex_NVIC_Config专用定义
#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6
#define GPIO_H 				7
#define GPIO_I 				8

#define FTIR   				1  		//下降沿触发
#define RTIR   				2  		//上升沿触发

//GPIO设置专用宏定义
#define GPIO_MODE_IN    	0		//普通输入模式
#define GPIO_MODE_OUT		1		//普通输出模式
#define GPIO_MODE_AF		2		//AF功能模式
#define GPIO_MODE_AIN		3		//模拟输入模式

#define GPIO_SPEED_2M		0		//GPIO速度2Mhz
#define GPIO_SPEED_25M		1		//GPIO速度25Mhz
#define GPIO_SPEED_50M		2		//GPIO速度50Mhz
#define GPIO_SPEED_100M		3		//GPIO速度100Mhz

#define GPIO_PUPD_NONE		0		//不带上下拉
#define GPIO_PUPD_PU		1		//上拉
#define GPIO_PUPD_PD		2		//下拉
#define GPIO_PUPD_RES		3		//保留 

#define GPIO_OTYPE_PP		0		//推挽输出
#define GPIO_OTYPE_OD		1		//开漏输出 

//GPIO引脚编号定义
#define PIN0				1<<0
#define PIN1				1<<1
#define PIN2				1<<2
#define PIN3				1<<3
#define PIN4				1<<4
#define PIN5				1<<5
#define PIN6				1<<6
#define PIN7				1<<7
#define PIN8				1<<8
#define PIN9				1<<9
#define PIN10				1<<10
#define PIN11				1<<11
#define PIN12				1<<12
#define PIN13				1<<13
#define PIN14				1<<14
#define PIN15				1<<15
//////////////////////////////////////////////////////////////////////////////////
u8 Sys_Clock_Set(u32 plln, u32 pllm, u32 pllp, u32 pllq);		//系统时钟设置

void Stm32_Clock_Init(u32 plln, u32 pllm, u32 pllp, u32 pllq); //时钟初始化
void Stm32_Clock_Init_HSI(u32 plln, u32 pllm, u32 pllp, u32 pllq);

void Sys_Soft_Reset(void);      							//系统软复位
void Sys_Standby(void);         							//待机模式
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);	//设置偏移地址
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);			//设置NVIC分组
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group); //设置中断
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM);				//外部中断配置函数(只对GPIOA~I)
void GPIO_AF_Set(GPIO_TypeDef* GPIOx, u8 BITx, u8 AFx);		//GPIO复用功能设置
void GPIO_Set(GPIO_TypeDef* GPIOx, u32 BITx, u32 MODE, u32 OTYPE, u32 OSPEED, u32 PUPD); //GPIO设置函数
//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址

void UnixT2Rtc(RTC_TIME_Str * ReadTime);
u32 Rtc2UnixT(RTC_TIME_Str * ReadTime);
void  Temper_Adc_Init(void);
void ReFresh_ADC(void);
void Get_SerialNum(void);

void LED_Init(void);	//初始化
void LED_ReFresh(void);
void getCRCtable2(u16 crcmask, u16 crcn, u32 ptable[]);

extern 	u32		crcTalbe[256];
u16 getCRC16(u8 *pinput, u16 inputlen, u32 ptable[]);


#define LED_OFF         0
#define LED_ON          1
#define LED_Flash_100Ms 2
#define LED_Flash_500Ms 10
#define LED_Flash_1s    20
#define LED_Flash_2s    40
#define LED_Flash_5s    100
#define LED_Flash_10s   200

extern u8  LED_Per[2];
extern u8  LED_Sat[2];

#define SYSSTALED LED_Per[0]
#define TKPSTALED LED_Per[1]

#define  DEBUGUART6 1

#endif











