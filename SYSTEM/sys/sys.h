#ifndef __SYS_H
#define __SYS_H
#include "stm32f4xx.h"

#include <rtl.h>

//crc16 ����ʽ����
#define 	CRC16		0x8408			//CRC-CCITT:x16+x12+x5+1  0x8408

#define Soft_Ver 	210
#define Hard_Ver 	110

#define KEEP_STA    PCout(8) 			//��ʱ�������IO

#define PPS_NAV		1
#define PPS_GPS		2
#define PPS_OFF		0

#define EXPUL_STA_OFF		0
#define EXPUL_STA_WAT		1
#define EXPUL_STA_IED		2
#define EXPUL_STA_PRT		3

#define ADC_CH_TEMP  	16 		 	  	//ͨ��16,�ڲ��¶ȴ�����ר��ͨ��	   	  

#define FLASH_SAVE_ADDR  0X080E0000

//////////////////////////////////////////////////////////////////////////
#define TK_PREOCXO          0       		//Ԥ��״̬***

#define TK_IDLE             1             	//����״̬*** ͬ��״̬

#define TK_RUNMEAS          2             	//����������״̬***
#define TK_COMMEAS          3             	//�������ɹ�״̬***
#define TK_OVEMEAS          4             	//��������ʱ״̬***

#define TK_RUNSYN_MEAS      5             	//ͬ������������״̬***
#define TK_COMSYN_MEAS      6             	//ͬ�������γɹ�״̬***
#define TK_OVESYN_MEAS      7             	//ͬ�������γ�ʱ״̬***

#define TK_RUNSYN_ADJ       8             	//ͬ������������״̬***
#define TK_COMSYN_ADJ       9             	//ͬ�����������״̬***
#define TK_OVESYN_ADJ       10				//ͬ�������γ�ʱ״̬***

#define TK_TIMKEEP          11   			//��ʱ״̬***
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
	u32     SYN_Time;			//�ϴ�ͬ��ʱ��
	u32    	ACQ_s;				//�ϴ�ͬ����������s
	u32    	ACQ_ns;				//�ϴ�ͬ����������ns
	u32    	ADJ_s;				//�ϴ�ͬ������s
	s32    	ADJ_ns;				//�ϴ�ͬ������s
} SYN_Info;


//����ϵͳ״̬�����ṹ
typedef struct
{
	u32     RunTime;                //����ʱ��
	u8      RunTime_50ms;   		//50msΪ��λ������ʱ��

	u8      TimeKeepSTA ;         	//��ѭ��״̬��
	s16     Temper;                	//��ǰоƬ�¶�
	u16     DigPot;             	//���ֵ�λ��ֵ��Ĭ��Ϊ469

	u8      TKS_Flag;        		//��ʱ�ȶ���ʶ

	u8      GPS_STA;         		//GPS״̬��ʾ
	u32     GPS_Time;               //GPSʱ��
	u8      GPS_SatN;               //GPS�ɼ�����
	double  GPS_longitude;     		//GPS����
	double  GPS_latitude;      		//GPSγ��

	u32     NATI_Time;				//����1PPSʱ��
	u32     NATI_Time_Prog;			//�����ɱ������ʱ��

	u32     Last_SYN_Time;			//�ϴ�ͬ��ʱ��
	u32     Last_ACQ_s;				//�ϴ�ͬ����������s
	u32     Last_ACQ_ns;			//�ϴ�ͬ����������ns
	u32     Last_ADJ_s;				//�ϴ�ͬ������s
	s32     Last_ADJ_ns;			//�ϴ�ͬ������s

	u8		Auto_SYN;				//��ͬ��ʹ��
	u8		Auto_OCXOADJ;			//�Ե�������ʹ��
	u8		Prog_Pul;				//�ɱ������ʹ��
	u8		Auto_KeepSIn;			//���л���ʱ״̬ʹ��
	u8		Meas_Pul;				//��������ʹ��
	u32		Prog_PerT;				//�ɱ����������


	u8		Messa_SYN;				//ͬ����Ϣ���ʹ��
	u8		Messa_1PPS;				//������������Ϣʹ��
	u8		Messa_PGP;				//�ɱ��������Ϣʹ��
	u8		Messa_GPS;				//GPS ����ת��ʹ��
} SYS_GSV ;

typedef struct
{
	u32     DigPot;             	//���ֵ�λ��ֵ��Ĭ��Ϊ469
	u32		Auto_SYN;				//��ͬ��ʹ��
	u32		Auto_OCXOADJ;			//�Ե�������ʹ��
	u32		Prog_Pul;				//�ɱ������ʹ��
	u32		Auto_KeepSIn;			//���л���ʱ״̬ʹ��
	u32		Meas_Pul;				//��������ʹ��
	u32		Prog_PerT;				//�ɱ����������
	u32		Messa_SYN;				//ͬ����Ϣ���ʹ��
	u32		Messa_1PPS;				//������������Ϣʹ��
	u32		Messa_PGP;				//�ɱ��������Ϣʹ��
	u32		Messa_GPS;				//GPS ����ת��ʹ��
} SYS_Para_Save ;


#define GPS_RNSS_OFF  0
#define GPS_RNSS_V    1
#define GPS_RNSS_A    2


//V1.1 ����GPS RMC GSV���ֱ���ת�����������ص�Ƭ��
//Timer.c�� Adj_OVETime = (PerTimer+1)/10/1000/1000 + 1 ;
//��ΪAdj_OVETime = (PerTimer+1)/10/1000/1000 + 2 ;��ֹ100ms����ʱ,һ���þͳ�ʱ�Ĵ���

#define MS2ON20NS(x) x*1000000/20
#define MS2ON5US(x)  x*1000/5

#define GATEGPSPPS  PAout(0)	// 

#define LED1        PAout(12)	// 
#define LED2 		PAout(11)	//

#define POWGPS      PCout(5)	// 
#define GPS_P_C   	PCin(9)  	//GPS������� 

#define PPS_SEL1  	PBout(2)	// 
#define PPS_SEL2  	PBout(1)	// 

//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_UCOS		0		//����ϵͳ�ļ����Ƿ�֧��UCOS


//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
//IO�ڵ�ַӳ��
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

//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����
//////////////////////////////////////////////////////////////////////////////////
//Ex_NVIC_Configר�ö���
#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6
#define GPIO_H 				7
#define GPIO_I 				8

#define FTIR   				1  		//�½��ش���
#define RTIR   				2  		//�����ش���

//GPIO����ר�ú궨��
#define GPIO_MODE_IN    	0		//��ͨ����ģʽ
#define GPIO_MODE_OUT		1		//��ͨ���ģʽ
#define GPIO_MODE_AF		2		//AF����ģʽ
#define GPIO_MODE_AIN		3		//ģ������ģʽ

#define GPIO_SPEED_2M		0		//GPIO�ٶ�2Mhz
#define GPIO_SPEED_25M		1		//GPIO�ٶ�25Mhz
#define GPIO_SPEED_50M		2		//GPIO�ٶ�50Mhz
#define GPIO_SPEED_100M		3		//GPIO�ٶ�100Mhz

#define GPIO_PUPD_NONE		0		//����������
#define GPIO_PUPD_PU		1		//����
#define GPIO_PUPD_PD		2		//����
#define GPIO_PUPD_RES		3		//���� 

#define GPIO_OTYPE_PP		0		//�������
#define GPIO_OTYPE_OD		1		//��©��� 

//GPIO���ű�Ŷ���
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
u8 Sys_Clock_Set(u32 plln, u32 pllm, u32 pllp, u32 pllq);		//ϵͳʱ������

void Stm32_Clock_Init(u32 plln, u32 pllm, u32 pllp, u32 pllq); //ʱ�ӳ�ʼ��
void Stm32_Clock_Init_HSI(u32 plln, u32 pllm, u32 pllp, u32 pllq);

void Sys_Soft_Reset(void);      							//ϵͳ��λ
void Sys_Standby(void);         							//����ģʽ
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);	//����ƫ�Ƶ�ַ
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);			//����NVIC����
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group); //�����ж�
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM);				//�ⲿ�ж����ú���(ֻ��GPIOA~I)
void GPIO_AF_Set(GPIO_TypeDef* GPIOx, u8 BITx, u8 AFx);		//GPIO���ù�������
void GPIO_Set(GPIO_TypeDef* GPIOx, u32 BITx, u32 MODE, u32 OTYPE, u32 OSPEED, u32 PUPD); //GPIO���ú���
//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ

void UnixT2Rtc(RTC_TIME_Str * ReadTime);
u32 Rtc2UnixT(RTC_TIME_Str * ReadTime);
void  Temper_Adc_Init(void);
void ReFresh_ADC(void);
void Get_SerialNum(void);

void LED_Init(void);	//��ʼ��
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











