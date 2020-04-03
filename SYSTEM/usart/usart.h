#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"

#define UARTRXBUFLEN 512

void UART6_SYS_init(u32 pclk2, u32 bound);
u16 Proc_RxByte_SYS(void);
void Proc_TxFrame_SYS(void);
u8 Proc_RxCMD_SYS(u8 * DataBuf);
void Send_STA_Sys(void);
void Send_Err(u8 ErrCMD, u8 * ErrData, u8 Len);
u16 Check_Para(u8 * Para_Data);
void Send_DelayAck_Sys(void);
void Send_Meas_Res(u32 AcqTimeDelay);
void Send_Meas_Adj_Res(void);
void Send_SoftVer(void);
void Send_SIGID(void);
void Send_GPS_C_V(u8* GPSdata);
void Send_NaviP(void);
void Send_Last_SYN_Table(SYN_Info * SYN_Info_p, u8 Next_SYN_Item);
char ProcPC_CMD(u8 New_CMD_Flag, u8 * New_Data );
void Send_Sys_NavPPS(void);
void Send_Sys_GPSPPS(void);
void Send_Sys_MeasPlu(u32 Mea_s, u32 Mea_ns);

__packed typedef struct
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
	s32   	GPS_longitude;     		//GPS经度
	s32   	GPS_latitude;      		//GPS纬度
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
} UART_SysSTAVarSt;

__packed typedef struct
{
	u8      TimeKeepSTA ;         	//主循环状态机
	u8      TKS_Flag;        		//守时稳定标识
	u32     NATI_Time;				//本机1PPS时戳
	u8      GPS_STA;         		//GPS状态标示
	u32     GPS_Time;               //GPS时间
	s32   	GPS_longitude;     		//GPS经度
	s32   	GPS_latitude;      		//GPS纬度
} UART_NaviVarSt;

//#define USART_REC_LEN  			200  	//定义最大接收字节数 200
//extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
//extern u16 USART_RX_STA;         		//接收状态标记
//void Uart6_init_ConSYSMCU(u32 pclk2,u32 bound);



__packed typedef struct
{
	/*
	time stamp
	*/
	u16 		uiYear;		//年(4位十进制数值):>1900
	u8			ucMonth;	//月:1-12
	u8			ucDay;		//天:1-31
	u8			ucWeek;   	//week 1-7     add week
	u8			ucHour;		//24小时制:0-23
	u8			ucMinute;	//分:0-59
	u8			ucSecond;	//秒:0-59
	//ms计数器<=1000
	u16 		usCounterms;
	/*
	坐标
	格式:N3939.7172520000,E116.9242730000
	*/
	char NorS;			//'N'/'n'北纬;'S'/'s'南纬
	char EorW;			//'E'/'e'东经;'W'/'w'西经
	//12byte
	double dLatitude;	//纬度  double float
	double dLongitude;  //经度
} SyncBoardMessage;

__packed typedef struct
{
	u8					fHead;
	u8					fType;
	SyncBoardMessage	messData;
	u16					crc16Check;
} SyFrame;


#endif

