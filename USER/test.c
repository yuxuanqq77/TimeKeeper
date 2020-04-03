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
////��ʼȫ��״̬��������
///////////////////////////////////////

//����ϵͳ״̬�ṹ
SYS_GSV		G_StaV_St ;

u8  		Err_Temp ;							//��ʱ�����ʶ

SYN_Info	Last_SYN_Table[5]   ;				//���5��ͬ����Ϣ��¼��
u8			Next_SYN_Item	= 0 ;				//ָ��һ�����ϵ�ͬ���������Ϣ

u32     SuccTimLast 	= 0 ;        			//���һ��ͬ��ʱ�䣬��¼���һ��ͬ���ı���ʱ��
s32     Correct_m_Last 	= 0 ;     				//���ͬ������ֵ

u32 	PerTimer 		= MS2ON20NS(1000) - 1;	//�������ڣ���20nsΪ��λ��Ĭ��Ϊ1s 50000000-1
u32 	PulW_NAT1P 		= MS2ON20NS(10);   		//��1������20nsΪ��λ��Ĭ��Ϊ10ms

u16 	SuccAdjNum 		= 0  ;          		//�ɹ�ͬ������

//������Ҫ�ı���
u8  	AcqTimeResFlag 	= 2 ;                	//���������־
u32 	AcqTime20ns 	= 0 ;              		//�����ʱ��ֵ����20nsΪ��λ
u32 	AcqTimeS		= 0 ;					//�����ʱ��ֵ����sΪ��λ
u32		AcqGPSTime		= 0 ;					//�����GPSʱ�䣬Ҳ����˵����ΪGPS��ʱ�������׼ȷ��
//��GPS������ǰ��ȥ����ʱ�������񵽵��Ǳ���ʱ��
//���û������ôAcqTimeS == AcqGPSTime ��AcqTime20ns == 0

//��������
u8  	AdjTimeResFlag 	= 2 ;      				//ͬ����־λ
s32 	Correct_m  		= 0;            		//ͬ������ֵ

u8  	Timer2_OnOff 	= 1 ;         			//��ʱ���ֶ��򿪻�ر�,���ڵ���

u32 	GPS_A_V_TimerOve		= 0 ;			//�ж�GPS�ź��Ƿ��ȶ��ı�־����
u32 	GPS_A_V_TimerOve50ms 	= 0 ;

u8		ProgP_SYN_Flag 			= 1 ;			//ͬ��������Ҫ����ͬ���ı�־λ
u8		NaviD_Send_Flag 		= 0 ;			//���ͱ��������屨�ı�־
u8		GPSD_Send_Flag			= 0 ;			//����GPS����֡��־

//�ⲿ����ʱ�����ɹ��ܱ���
u8		exPulMeasFlag		= EXPUL_STA_OFF ;	//�ⲿ�����־
u32		exPul_TimerOve		= 0 ;				//�ⲿ���屣���������
u8 		exPul_TimerOve50ms 	= 0 ;
u32		exPulMeas_ns		= 0 ;				//�ⲿ���岶����
u32		exPulMeas_s			= 0 ;

//����ʧ����
SYS_Para_Save Try_ParaVar ;						//����ʧ������ʱʹ�ýṹ

u32 Device_SN0 = 0 ;
u32 Device_SN1 = 0 ;
u32 Device_SN2 = 0 ;

///////////////////////////////////////
////����ȫ��״̬��������
///////////////////////////////////////


//////////////////////////////////////////////////
/////����PPS���ͨ�� ����pps��GPS pps ���߲��������
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
//����pps���ͨ��
void refPPSSel(void)
{
	if(G_StaV_St.GPS_STA  == GPS_RNSS_A)	//���GPS�ź���Ч�������GPS��PPS
	{
		if(PPS_Sel_Flag != PPS_GPS)
			SetPPSSel(PPS_GPS);
	}
	else if(G_StaV_St.TKS_Flag == 1)		//���gps��Ч�����Ǳ���pps���������������pps
	{
		if(PPS_Sel_Flag != PPS_NAV)
			SetPPSSel(PPS_NAV);
	}
	else									//���򣬲����
	{
		if(PPS_Sel_Flag != PPS_OFF)
			SetPPSSel(PPS_OFF);
	}
}
//////////////////////////////////////////////////
/////��ʼ��PPS���ͨ��ѡ��
//////////////////////////////////////////////////
void initPPSSel(void)
{
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;														//ʹ��PORTBʱ��
	GPIO_Set(GPIOB, PIN1 | PIN2, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); 	//
	SetPPSSel(PPS_OFF);
}

//////////////////////////////////////////////////
/////��ʼ����ʱ�������
//////////////////////////////////////////////////
void initKeepSta(void)
{
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;														//ʹ��PORTCʱ��

	KEEP_STA = 0;
	GPIO_Set(GPIOC, PIN8, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); 		//
}


//////////////////////////////////////
//��ʼ��ȫ��״̬�ṹ����
void InitGlbSt(void)
{
	//��ȡFlash�еĲ���
	//Get_Para(&Try_ParaVar);
	AT24CXX_Init();
	Get_Para_IIC(&Try_ParaVar);								//��iic eeprom�л�ȡ����

	//ȫ�ֱ�����ʼ��
	G_StaV_St.RunTime         		= 0;                	//����ʱ��
	G_StaV_St.RunTime_50ms  		= 0;                	//50msΪ��λ������ʱ��

	G_StaV_St.TimeKeepSTA         	= TK_PREOCXO ;         	//��ѭ��״̬��
	G_StaV_St.Temper              	= 0;                	//��ǰоƬ�¶�
	G_StaV_St.DigPot              	= Try_ParaVar.DigPot ;	//���ֵ�λ��ֵ��Ĭ��Ϊ469		*��Ҫ�洢��flash

	G_StaV_St.TKS_Flag       		= 0 ;               	//��ʱ�ȶ���ʶ

	G_StaV_St.GPS_STA             	= GPS_RNSS_OFF ;		//GPS״̬��ʾ,����ʱ�ر�GPS��Դ
	G_StaV_St.GPS_Time            	= 0 ;               	//GPSʱ��
	G_StaV_St.GPS_SatN            	= 0 ;               	//GPS�ɼ�����
	G_StaV_St.GPS_longitude       	= 0.0f;          		//GPS����
	G_StaV_St.GPS_latitude        	= 0.0f;          		//GPSγ��

	G_StaV_St.NATI_Time  			= 0;					//����1PPSʱ��
	G_StaV_St.NATI_Time_Prog  		= 0;					//�����ɱ������ʱ��

	G_StaV_St.Last_SYN_Time       	= 0 ;					//�ϴ�ͬ��ʱ��
	G_StaV_St.Last_ACQ_s    		= 0 ;					//�ϴ�ͬ���������s
	G_StaV_St.Last_ACQ_ns   		= 0 ;					//�ϴ�ͬ���������ns
	G_StaV_St.Last_ADJ_s          	= 0 ;					//�ϴ�ͬ������s
	G_StaV_St.Last_ADJ_ns         	= 0 ;					//�ϴ�ͬ������s

	G_StaV_St.Auto_SYN				= Try_ParaVar.Auto_SYN ;			//��ͬ��ʹ��					*��Ҫ�洢��flash
	G_StaV_St.Auto_OCXOADJ			= Try_ParaVar.Auto_OCXOADJ ;		//�Ե�������ʹ��				*��Ҫ�洢��flash
	G_StaV_St.Prog_Pul				= Try_ParaVar.Prog_Pul ;			//�ɱ������ʹ��				*��Ҫ�洢��flash
	G_StaV_St.Auto_KeepSIn			= Try_ParaVar.Auto_KeepSIn ;		//���л���ʱ״̬ʹ��			*��Ҫ�洢��flash
	G_StaV_St.Meas_Pul				= Try_ParaVar.Meas_Pul ;			//��������ʹ��				*��Ҫ�洢��flash
	G_StaV_St.Prog_PerT				= Try_ParaVar.Prog_PerT; 			//�ɱ����������				*��Ҫ�洢��flash

	G_StaV_St.Messa_SYN				= Try_ParaVar.Messa_SYN ;			//ͬ����Ϣ���ʹ��			*��Ҫ�洢��flash
	G_StaV_St.Messa_1PPS			= Try_ParaVar.Messa_1PPS ;			//������������Ϣʹ��			*��Ҫ�洢��flash
	G_StaV_St.Messa_PGP				= Try_ParaVar.Messa_PGP ;			//�ɱ��������Ϣʹ��			*��Ҫ�洢��flash
	G_StaV_St.Messa_GPS				= Try_ParaVar.Messa_GPS ;			//GPS ����ת��ʹ��			*��Ҫ�洢��flash

	//��ʼ��CRC16��
	getCRCtable2( CRC16, 16, crcTalbe);
}
//////////////////////////////////////
//��ʼ��Ӳ��
void InitHardware(void)
{
	//GPS��������ſ��źţ������ʹ�����أ������������źž���
	//GPS�������ſ�
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;													//ʹ��PORTCʱ��
	GPIO_Set(GPIOA, PIN0, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); 	//
	GATEGPSPPS = 0 ;
	GATEGPSPPS = 1 ;

	//��ʼ�����¾���ѹ�����ֵ�λ��
	DIGPOT_Init();
	DIGPOT_Write(G_StaV_St.DigPot);

	//��ʼ��GPSʱ�ӣ�����Ϊ�˵��Է���Ĭ�ϴ�
	GPS_Pow_init();
	GPS_Pow_Re();

	//��ʼ��LEDָʾ��
	LED_Init();

	//��ʼ��ADC���¶Ȳɼ�
	init_TMP121();

	delay_ms(250);

	//����ʱ�ӳ�ʼ��
	TIM6_50ms_Init();

	//��ʼ�������ص�Ƭ���Ĵ��ڣ�������Ҫ���ڲ���Ӳ����������������,����������Э��
	UART6_SYS_init(25, 115200);
	Send_STA_Sys();

	//��ʼ��GPS����
	UART1_GPS_init(25, 9600);

	//��ʼ��ppsͨ��ѡ��IO
	initPPSSel();
	//��ʼ����ʱ״̬���IO
	initKeepSta();
}
/////////////////////////////////
////����ÿ������Ҫ���������
void ProcPer1s(void)
{
	s16		TMP121_Raw		= 0 ;
	float 	Tempr_f			= 0.0f;		//�¶��˲���

	//�򿪻�ر��ⲿ�����������
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

	//�¶Ȳ���
	TMP121_Raw  = Get_TMP_Raw();									//����Ϊ�ⲿ���¶ȴ�����
	Tempr_f		= TMP121_Raw * 0.0625f;
	G_StaV_St.Temper	=	(s16)(Tempr_f * 10);					//��10ȡ��������S16�洢����Ҫ��ʱ�򣬡�10�������϶�

	GPS_Pow_Re();													//ˢ��GPS��Դ����
	refPPSSel();													//ˢ��PPSͨ��ѡ��

	//��GPS����ͻȻû�ˣ���GPS��״̬����Ϊ��Ч
	if(G_StaV_St.GPS_STA == GPS_RNSS_A)								//��GPS��Ч��״̬�£���ʱ��û���յ�GPS���ģ���Ϊ��Ч
	{
		if( CheckTimerOve(GPS_A_V_TimerOve, GPS_A_V_TimerOve50ms) == 1)
			G_StaV_St.GPS_STA = GPS_RNSS_V ;
	}
}

//////////////////////////////////////
//������
int main(void)
{
	u8  Temp_AcqTimeRes ;				//���ڻ���ɼ�ʱ���־λ������״̬����
	u8  Temp_AdjTimeRes ;				//���ڻ������ʱ���־λ������״̬����

	u8 	New_CMD_Flag = 0;				//�µĴ������������λ��
	u8 	New_Data[301];					//�µĴ�������ģ�������������λ��

	u32  SYN_TimerOveSec 	= 0; 		//ͬ������ʹ�õĳ�ʱ����
	u8   SYN_TimerOve50ms	= 0;

	u32  Last_SYN_Time_ed	= 0;		//��һ�������ж��Ƿ�������ʱ��

	u8		Per1s_Flag 		= 0;		//ÿ1s��һ�εı�־����

	double	Syn_Err 		= 0.0f;		//ͬ��������

	delay_init(10);						//�������ʱһ��Ҫ�ӣ��ȴ������ȶ������¾����ʼ�ļ���ms��Ҫ�ȶ�һ��
	delay_ms(500);
	delay_ms(500);

	//�ڲ�RC��50MHz������ʹ��
	//Stm32_Clock_Init_HSI(100, 8, 4, 7); 	//����ʱ��,50Mhz
	//APB1 25MHz TIM2/3/4/5/6/7 50MHz
	//APB2 25MHz TIM1/8/9/10/11 50MHz

	//���¾���50MHz
	Stm32_Clock_Init(100, 10, 2, 7);   		//����ʱ��,50Mhz
	//APB1 25MHz TIM2/3/4/5/6/7 50MHz
	//APB2 25MHz TIM1/8/9/10/11 50MHz

	delay_init(50);                     	//��ʱ��ʼ��
	delay_ms(250); 							//FLASH����֮ǰ��Ҫ��ʱ

	InitGlbSt();							//��ʼ���ṹ����
	InitHardware();							//��ʼ��Ӳ��

	//��ʼ����������TIM2
	TIM2_KEEP_Init(PerTimer, PulW_NAT1P);				//��ʼ����ʱ��2������ʱ���������ʱ��ά���ű���1s���壬NAT1S
	//ģ������й��ܻ��������������ʱ��
	//CH3->����������
	//CH2->����GPS������
	//CH1->���ڰ����ڵ����ж�
	//CH4->�����ⲿ��������˿�

	delay_ms(250);   									//�ڽ�����ѭ��֮ǰ������ʾ250ms���ȶ�һ��

	SYSSTALED	=	LED_Flash_100Ms;					//Ԥ��״̬
	TKPSTALED	= 	LED_Flash_100Ms;

	//��ѭ����ʼ����Ҳ����ͣ����
	while(1)											//��ѭ���Ӵ˴���ʼ
	{
		//ÿ�����һ��,��Ҫ��ADC��ѹ�ɼ�ת��������������������ܻ�Ҫ����
		if( G_StaV_St.RunTime_50ms >= 9 && Per1s_Flag == 0 )
		{
			Per1s_Flag = 1;														//ȷ����ÿһ�룬ֻ�����һ��
			ProcPer1s();														//ÿ������Ҫ���������
		}

		//ÿ�봦�������

		//���� �ⲿ����������� ����
		if(G_StaV_St.Meas_Pul == 1)
		{
			if(exPulMeasFlag == EXPUL_STA_IED)
			{
				Send_Sys_MeasPlu(exPulMeas_s, exPulMeas_ns);
				exPulMeasFlag = EXPUL_STA_PRT ;									//�������״̬
			}
			else if(exPulMeasFlag == EXPUL_STA_PRT)
			{
				if( CheckTimerOve(exPul_TimerOve, exPul_TimerOve50ms) == 1)		//�����������
				{
					exPulMeasFlag = EXPUL_STA_WAT ;								//���´��ⲿ���岶��
					TIM2_ExPul_Meas_Start();
				}
			}
		}

		//���� PPS��Ϣת������ ����
		if(G_StaV_St.Messa_1PPS == 1)
		{
			//������������屨�ķ��䣬�������µ���������������Ѿ�������GPS��Ч�����������屨��
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
			else			//GPS��Ч������û������
			{
				NaviD_Send_Flag = 0 ;
				GPSD_Send_Flag 	= 0 ;
			}
		}
		else				//����ر�pps������Ϣ  ʲô��������
		{
			NaviD_Send_Flag = 0 ;
			GPSD_Send_Flag 	= 0 ;
		}

		//GPS���Ķ��㴦��
		Prorc_GPS_Top();

		//����ԭʼ�Ľ��ջ�����δ�����ֽڣ���������Ч����֡
		Proc_RxByte_SYS();
		//ȡ�����ϵ�һ��������Ч֡
		New_CMD_Flag = Proc_RxCMD_SYS(New_Data);
		New_CMD_Flag = ProcPC_CMD(New_CMD_Flag, New_Data);		//��������������
		//��S���л�״̬�ڸ���״̬�ﴦ��
		//�����ڷ�������,�������Ч֡��Ҫ�����ҷ���DMAû���ڹ���,
		//��ô�����ϵ�һ֡ʹ��DMA���ͳ�ȥ.����ֱ���˳�.
		Proc_TxFrame_SYS();

		////////////////////////////////////////////////////////////////////////////
		/////////
		/////////    ��ѭ����״̬���Ӵ˿�ʼ
		/////////
		////////////////////////////////////////////////////////////////////////////
		switch(G_StaV_St.TimeKeepSTA)
		{
			//Ԥ��״̬
			case TK_PREOCXO:

				if(G_StaV_St.RunTime > 120)
				{
					SYSSTALED	=	LED_Flash_1s;
					G_StaV_St.TimeKeepSTA = TK_IDLE;
					Send_STA_Sys();
				}

				if(New_CMD_Flag == 'S')							//�ֶ�����ͬ��״̬
				{
					if(New_Data[0] == 'S')
					{
						SYSSTALED	=	LED_Flash_1s;
						G_StaV_St.TimeKeepSTA = TK_IDLE;
						Send_STA_Sys();
					}
				}

				break;

			//����״̬���ֳ�ͬ��״̬
			case TK_IDLE:                                  		//����״̬***�ĵ��г�Ϊ��ͬ��״̬��

				//��ͬ�������㣬ͬ�����������ʱ����ʼһ�β�������ͬ������
				if( G_StaV_St.Auto_SYN == 1 && G_StaV_St.GPS_STA == GPS_RNSS_A )
				{
					//��������һ��  ͬ��������
					if( (	G_StaV_St.GPS_Time % 86400) % (1200 / 10) == (1200 - 10) / 10 				//����ͬ�������������弴������,2min(60s)������������ǰһ��
							&& G_StaV_St.GPS_Time >= (G_StaV_St.Last_SYN_Time + (1200 - 10) / 10) 		//��ǰ��GPSʱ����ϴ�ͬ��ʱ���һ��ͬ��������1s,��ֹ��ν���ͬ��
							&& CheckTimerOve(SYN_TimerOveSec, SYN_TimerOve50ms) == 1 					//�ϴ�ͬ������2s����,�ſɽ���ͬ��
					  )
					{
						AcqGPSTime		= G_StaV_St.GPS_Time + 1;	//��һ��GPS�������ʱ������������������һ���������ǰ�ؾ�������Ҫ����ͬ��������
						AcqTimeResFlag 	= 0 ;						//��ʼ��������־λ
						TIM2_Meas_Start();              			//������������
						G_StaV_St.TimeKeepSTA = TK_RUNSYN_MEAS ;  	//���������ͬ���еĲ���״̬,������
					}
				}
				else if( G_StaV_St.Auto_SYN == 0 && G_StaV_St.GPS_STA == GPS_RNSS_A )					//ֻ��������ͬ��
				{
					//��������һ��  ������
					if( (	G_StaV_St.GPS_Time % 86400) % (1200 / 10) == (1200 - 10) / 10 				//����ͬ�������������弴������,2min(60s)������������ǰһ��
							&& G_StaV_St.GPS_Time >= (G_StaV_St.Last_SYN_Time + (1200 - 10) / 10) 		//��ǰ��GPSʱ����ϴ�ͬ��ʱ���һ��ͬ��������1s,��ֹ��ν���ͬ��
							&& CheckTimerOve(SYN_TimerOveSec, SYN_TimerOve50ms) == 1 					//�ϴ�ͬ������2s����,�ſɽ���ͬ��
					  )
					{
						AcqGPSTime		= G_StaV_St.GPS_Time + 1;									//��һ��GPS�������ʱ������������������һ���������ǰ�ؾ�������Ҫ��������������
						AcqTimeResFlag 	= 0 ;            											//��ʼ��������־λ
						TIM2_Meas_Start();              											//������������
						G_StaV_St.TimeKeepSTA = TK_RUNMEAS ;      									//���뵥����ͬ���������ʱ��״̬
					}
				}

				//�������һ�ε�ͬ��������ж��Ƿ��������
				if( 	Last_SYN_Time_ed != G_StaV_St.Last_SYN_Time 		//���µ�ͬ���㣬�������ж��Ƿ�����
						&& 	G_StaV_St.RunTime > 900 						//�����ϵ� 900s �����ж��Ƿ����������Ǻ��¾���
						&& 	G_StaV_St.Last_SYN_Time != 0)					//���û��ͬ��1�Σ��������ж�
				{
					Last_SYN_Time_ed = G_StaV_St.Last_SYN_Time ;

					Syn_Err = (double)G_StaV_St.Last_SYN_Time - (double)G_StaV_St.Last_ACQ_s ;

					Syn_Err = Syn_Err - ((double)G_StaV_St.Last_ACQ_ns) * 20.0f / 1000000000.0f ;

					if(Syn_Err < 0)
						Syn_Err = -Syn_Err ;

					if(Syn_Err < 0.000001)					//���С��1us����Ϊ����
					{
						TKPSTALED	= 	LED_Flash_2s;

						G_StaV_St.TKS_Flag 	= 1 ;
						KEEP_STA 			= 1;
					}
					else
					{
						TKPSTALED	= 	LED_Flash_100Ms;	//û������

						G_StaV_St.TKS_Flag 	= 0 ;
						KEEP_STA 			= 0;
					}
				}

				if(G_StaV_St.TimeKeepSTA == TK_IDLE)		//�����п����л�״̬�����ԣ��ȱ�֤��ǰ��û�н�������ͬ�����߲�����״̬
				{
					if(G_StaV_St.Auto_KeepSIn == 1 && G_StaV_St.TKS_Flag == 1 )	//�Զ��л�����ʱ״̬
					{
						SYSSTALED	=	LED_Flash_2s;
						G_StaV_St.TimeKeepSTA = TK_TIMKEEP ;
						Send_STA_Sys();
					}

					if(New_CMD_Flag == 'S')										//�ֶ�������ʱ״̬
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

			//��ʱ״̬����״̬�²�ͬ����GPS�ر�
			case TK_TIMKEEP:

				if(New_CMD_Flag == 'S')											//�ֶ�����ͬ��״̬
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

			case TK_RUNMEAS:                        		//����������״̬***

				//�����״̬�£��ȵ�������ʱ�ӻ��߳�ʱ
				Temp_AcqTimeRes = AcqTimeResFlag ;    		//ȡ������,��ֹ��if����г��ֱ������ж��޸ĵĴ���

				if( Temp_AcqTimeRes == 1 )            		//�����ɹ�
				{
					G_StaV_St.TimeKeepSTA = TK_COMMEAS ;
				}
				else if( Temp_AcqTimeRes == 2 )       		//������ʱ
				{
					G_StaV_St.TimeKeepSTA = TK_OVEMEAS ;
				}
				else if( Temp_AcqTimeRes == 0 )       		//����������
					G_StaV_St.TimeKeepSTA = TK_RUNMEAS ;
				else
				{
					//�쳣�������˻ؿ���
					Err_Temp = 2;
					Send_Err('M', &Err_Temp, 1);
					G_StaV_St.TimeKeepSTA = TK_IDLE ;
				}

				break;

			case TK_COMMEAS:                        		//���������״̬***

				G_StaV_St.Last_SYN_Time						=	AcqGPSTime ;		//�ϴ�ͬ��ʱ��
				G_StaV_St.Last_ACQ_s						=	AcqTimeS ;			//�ϴ�ͬ������s
				G_StaV_St.Last_ACQ_ns						=	AcqTime20ns ;		//�ϴ�ͬ������ns
				G_StaV_St.Last_ADJ_s						=	0 ;					//�ϴ�ͬ������s,û�е�����ֻ�ǲ���������д0
				G_StaV_St.Last_ADJ_ns						=	0 ;					//�ϴ�ͬ������s

				//���浽���5��ͬ����Ϣ����ṹ
				Last_SYN_Table[Next_SYN_Item].SYN_Time		=	AcqGPSTime ;		//�ϴ�ͬ��ʱ��
				Last_SYN_Table[Next_SYN_Item].ACQ_s			=	AcqTimeS ;			//�ϴ�ͬ����������s
				Last_SYN_Table[Next_SYN_Item].ACQ_ns		=	AcqTime20ns ;		//�ϴ�ͬ����������ns
				Last_SYN_Table[Next_SYN_Item].ADJ_s			=	0 ;					//�ϴ�ͬ������s
				Last_SYN_Table[Next_SYN_Item].ADJ_ns		=	0 ;					//�ϴ�ͬ������s
				Next_SYN_Item++;
				Next_SYN_Item %= 5;

				SYN_TimerOve50ms	= G_StaV_St.RunTime_50ms ;    					//�յ������2���ڲ��ٽ���ͬ������
				SYN_TimerOveSec 	= G_StaV_St.RunTime + 2;                		//��Ҫ�Ƿ�ֹ,ͬ�������ܿ�ͽ���,�������ڶ���ͬ������

				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				//�ϱ����
				if(G_StaV_St.Messa_SYN == 1)
				{
					Send_Meas_Adj_Res();
				}

				break;

			case TK_OVEMEAS:                        	//��������ʱ״̬***

				//�ϱ�����ʱ
				Err_Temp = 1;
				Send_Err('M', &Err_Temp, 1);
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;

			case TK_RUNSYN_MEAS:                    	//ͬ������������״̬***

				//�����״̬�£��ȵ�������ʱ�ӻ��߳�ʱ
				Temp_AcqTimeRes = AcqTimeResFlag ;    	//ȡ������,��ֹ��if����г��ֱ������ж��޸ĵĴ���

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
					//�쳣�������˻ؿ���
					Err_Temp = 2;
					Send_Err('S', &Err_Temp, 1);
					G_StaV_St.TimeKeepSTA = TK_IDLE ;
				}

				break;

			case TK_COMSYN_MEAS:                    					//ͬ�����������״̬***

				//���ݲ�����������������ֵ
				Correct_m = AdjCalc(PerTimer, AcqTime20ns, PulW_NAT1P);	//���ݲ���ֵ���������

				//�򿪵������г���
				AdjTimeResFlag = 0 ;
				TIM2_Adj_Start();

				G_StaV_St.TimeKeepSTA = TK_RUNSYN_ADJ ;           		//�������״̬

				break;

			case TK_OVESYN_MEAS:                        				//ͬ�������γ�ʱ״̬***
				//�ϱ�����ʱ
				Err_Temp = 1;
				Send_Err('S', &Err_Temp, 1);							//�����γ�ʱ
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;

			case TK_RUNSYN_ADJ:                         				//ͬ������������״̬***

				//�����״̬�£��ȵ������������߳�ʱ
				Temp_AdjTimeRes = AdjTimeResFlag ;      				//ȡ������,��ֹ��if����г��ֱ������ж��޸ĵĴ���

				if( Temp_AdjTimeRes == 1)//��������
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
					//�쳣�������˻ؿ���
					Err_Temp = 3;
					Send_Err('S', &Err_Temp, 1);						//�������쳣,����ط��쳣�Ŀ����ԱȽ�С
					G_StaV_St.TimeKeepSTA = TK_IDLE ;
				}

				break;

			case TK_COMSYN_ADJ:											//ͬ�������γɹ�״̬***

				G_StaV_St.Last_SYN_Time						=	AcqGPSTime ;		//�ϴ�ͬ��ʱ��
				G_StaV_St.Last_ACQ_s						=	AcqTimeS ;			//�ϴ�ͬ������s
				G_StaV_St.Last_ACQ_ns						=	AcqTime20ns ;		//�ϴ�ͬ������ns
				G_StaV_St.Last_ADJ_s						=	AcqGPSTime ;		//�ϴ�ͬ������s��������ֱ�����ó�GPS��ʱ�䣬Ϊ�˼򵥺���֮ǰ�������
				G_StaV_St.Last_ADJ_ns						=	Correct_m ;			//�ϴ�ͬ������s

				//���浽���5��ͬ����Ϣ����ṹ
				Last_SYN_Table[Next_SYN_Item].SYN_Time		=	AcqGPSTime ;		//�ϴ�ͬ��ʱ��
				Last_SYN_Table[Next_SYN_Item].ACQ_s			=	AcqTimeS ;			//�ϴ�ͬ����������s
				Last_SYN_Table[Next_SYN_Item].ACQ_ns		=	AcqTime20ns ;		//�ϴ�ͬ����������ns
				Last_SYN_Table[Next_SYN_Item].ADJ_s			=	AcqGPSTime;			//�ϴ�ͬ������s
				Last_SYN_Table[Next_SYN_Item].ADJ_ns		=	Correct_m;			//�ϴ�ͬ������s
				Next_SYN_Item++;
				Next_SYN_Item %= 5;

				SYN_TimerOve50ms	= G_StaV_St.RunTime_50ms ;    					//�յ������2���ڲ��ٽ���ͬ������
				SYN_TimerOveSec 	= G_StaV_St.RunTime + 2;						//��Ҫ�Ƿ�ֹ,ͬ�������ܿ�ͽ���,�������ڶ���ͬ������

				SuccAdjNum ++  ;

				G_StaV_St.TimeKeepSTA		= 	TK_IDLE ;

				//�ϱ���������������͵�����ֵ
				if(G_StaV_St.Messa_SYN == 1)
				{
					Send_Meas_Adj_Res();
				}

				break;

			case TK_OVESYN_ADJ:                         //ͬ�������γ�ʱ״̬***
				//�ϱ�����ʱ
				Err_Temp = 4;
				Send_Err('S', &Err_Temp, 1);            //�����γ�ʱ
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;

			default:                                    //δ�����״̬***

				//�ϱ�δ֪״̬����
				Err_Temp = 1;
				Send_Err('Q', &Err_Temp, 1);            //δ֪״̬����
				G_StaV_St.TimeKeepSTA = TK_IDLE ;

				break;
		}

		//����1s��־����������������1�κ���һ�벻���ٽ�����
		if( G_StaV_St.RunTime_50ms == 0 && ( Per1s_Flag == 1 ) )
		{
			Per1s_Flag = 0 ;
		}
	}
}
//

