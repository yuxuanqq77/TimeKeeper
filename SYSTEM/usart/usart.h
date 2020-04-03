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
	u32     RunTime;                //����ʱ��
	u8      RunTime_50ms;   		//50msΪ��λ������ʱ��
	u8      TimeKeepSTA ;         	//��ѭ��״̬��
	s16     Temper;                	//��ǰоƬ�¶�
	u16     DigPot;             	//���ֵ�λ��ֵ��Ĭ��Ϊ469
	u8      TKS_Flag;        		//��ʱ�ȶ���ʶ
	u8      GPS_STA;         		//GPS״̬��ʾ
	u32     GPS_Time;               //GPSʱ��
	u8      GPS_SatN;               //GPS�ɼ�����
	s32   	GPS_longitude;     		//GPS����
	s32   	GPS_latitude;      		//GPSγ��
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
} UART_SysSTAVarSt;

__packed typedef struct
{
	u8      TimeKeepSTA ;         	//��ѭ��״̬��
	u8      TKS_Flag;        		//��ʱ�ȶ���ʶ
	u32     NATI_Time;				//����1PPSʱ��
	u8      GPS_STA;         		//GPS״̬��ʾ
	u32     GPS_Time;               //GPSʱ��
	s32   	GPS_longitude;     		//GPS����
	s32   	GPS_latitude;      		//GPSγ��
} UART_NaviVarSt;

//#define USART_REC_LEN  			200  	//�����������ֽ��� 200
//extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
//extern u16 USART_RX_STA;         		//����״̬���
//void Uart6_init_ConSYSMCU(u32 pclk2,u32 bound);



__packed typedef struct
{
	/*
	time stamp
	*/
	u16 		uiYear;		//��(4λʮ������ֵ):>1900
	u8			ucMonth;	//��:1-12
	u8			ucDay;		//��:1-31
	u8			ucWeek;   	//week 1-7     add week
	u8			ucHour;		//24Сʱ��:0-23
	u8			ucMinute;	//��:0-59
	u8			ucSecond;	//��:0-59
	//ms������<=1000
	u16 		usCounterms;
	/*
	����
	��ʽ:N3939.7172520000,E116.9242730000
	*/
	char NorS;			//'N'/'n'��γ;'S'/'s'��γ
	char EorW;			//'E'/'e'����;'W'/'w'����
	//12byte
	double dLatitude;	//γ��  double float
	double dLongitude;  //����
} SyncBoardMessage;

__packed typedef struct
{
	u8					fHead;
	u8					fType;
	SyncBoardMessage	messData;
	u16					crc16Check;
} SyFrame;


#endif

