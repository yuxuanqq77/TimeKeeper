#include "sys.h"
#include "GPS.h"
#include "timer.h"
#include "string.h"
#include "usart.h"

extern u32	GPS_A_V_TimerOve ;
extern u32	GPS_A_V_TimerOve50ms ;
extern u8	GPSD_Send_Flag;

//���ڽ������ݻ�������DMA�Զ���䣬ѭ������
__align(4) u8  U1_GPS_RxBuf[UARTRXBUFLEN_U1] ;
u16 U1_GPS_Rx_ProcPos = 0;          //ָ��û�д�����ֽ�
u16 U1_GPS_Rx_GetPos  = 0;          //ָ��û�б������ֽڣ��ڽ��մ������и���

//������������Ч֡
__align(4) u8  U1_GPS_RxFrame[20][512] ;
u16 U1_GPS_Rx_ProcFrame = 0;        //ָ��û�д������Ч֡
u16 U1_GPS_Rx_GetFrame  = 0;        //ָ��û�б����������Ŀ�֡


//////////////////////////////////////
////  UART1 RX ʹ�õ�DMA
////  DMA2 Stream2 Ch4 ��ʼ��
void UART1_RX_DMA_Config(void)
{
	DMA_Stream_TypeDef * DMA_Streamx ;
	DMA_TypeDef *DMAx;
	u8 streamx;
	u8 chx ;

	DMA_Streamx = DMA2_Stream2 ;
	DMAx        = DMA2 ;
	streamx     = 2 ;
	chx         = 4 ;

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; 					//DMA2ʱ��ʹ��

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 						//�ر�DMA����

	while(1)
	{
		if( (DMA_Streamx->CR & DMA_SxCR_EN) == 0 )
			break;
	}

	streamx = (((u32)DMA_Streamx - (u32)DMAx) - 0X10) / 0X18;		//�õ�streamͨ����

	if(streamx >= 6)
		DMAx->HIFCR |= 0X3D << (6 * (streamx - 6) + 16);					//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx >= 4)
		DMAx->HIFCR |= 0X3D << 6 * (streamx - 4);    					//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx >= 2)
		DMAx->LIFCR |= 0X3D << (6 * (streamx - 2) + 16);					//���֮ǰ��stream�ϵ������жϱ�־
	else
		DMAx->LIFCR |= 0X3D << 6 * streamx;							//���֮ǰ��stream�ϵ������жϱ�־

	DMA_Streamx->PAR  = (u32) & (USART1->DR);		  			//DMA�����ַ
	DMA_Streamx->M0AR = (u32)U1_GPS_RxBuf ;		  			//DMA �洢��0��ַ
	DMA_Streamx->NDTR = UARTRXBUFLEN_U1;		        	//
	DMA_Streamx->CR   = 0;			                		//��ȫ����λCR�Ĵ���ֵ

	DMA_Streamx->CR  &= ~DMA_SxCR_DIR;		    			//���赽�洢��ģʽ
	DMA_Streamx->CR  |= DMA_SxCR_CIRC;	    				//ѭ��ģʽ
	DMA_Streamx->CR  &= ~DMA_SxCR_PINC;		  				//���������ģʽ
	DMA_Streamx->CR  |= DMA_SxCR_MINC;		  				//�洢������ģʽ
	DMA_Streamx->CR  &= ~DMA_SxCR_PSIZE;		  				//�������ݳ���:8λ
	DMA_Streamx->CR  &= ~DMA_SxCR_MSIZE;		  				//�洢�����ݳ���:8λ
	DMA_Streamx->CR  &= ~DMA_SxCR_PL;		    			//�����ȼ�
	DMA_Streamx->CR  &= ~DMA_SxCR_PBURST;					//����ͻ�����δ���
	DMA_Streamx->CR  &= ~DMA_SxCR_MBURST;					//�洢��ͻ�����δ���

	DMA_Streamx->CR  |= (u32)chx << 25;     //ͨ��ѡ��
	DMA_Streamx->CR  &= ~DMA_SxCR_TCIE ;    //��Ҫ�ж�
	DMA_Streamx->CR  |= DMA_SxCR_EN;        //��DMA
}


//////////////////////////////////////////////////////////////////////////////////
//// UART1��GPS�Ĵ��ڳ�ʼ��
//// ����һ���ܸ߿Ƽ��Ĵ�������
////
//// ����ʹ��DMA���뻷�λ�������DMAѭ������,
//// ��DMA�Ĵ�������Ĵ������Ѿ�������ֽ�λ��,
//// �ж��Ƿ�����Ҫ������ֽڡ�
//// ������ֽ�û�д����Ͷ��������봮��״̬��ѭ��
//// ����������Ч֡��������Ч֡���������ȴ���ѭ������
////
//// ���Ҳʹ��DMA��ͨ�����¼��뷢������������뷢��������������
//// �ж��Ƿ���Ҫ��ʼ���͡�����У�������֡��DMA���ͳ�ȥ��
////
//////////////////////////////////////////////////////////////////////////////////
void UART1_GPS_init(u32 pclk2, u32 bound)
{
	float temp;

	u16 mantissa;
	u16 fraction;
	UART1_RX_DMA_Config();                  			//����ѭ������DMA��������
	temp = (float)(pclk2 * 1000000) / (bound * 16); 	//�õ�USARTDIV@OVER8=0
	mantissa = temp;				       				//�õ���������
	fraction = (temp - mantissa) * 16;            		//�õ�С������@OVER8=0
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN ;   	//ʹ��GPIOA��ʱ��
	RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;  	//ʹ�ܴ���1ʱ��
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;  	//��λ����1
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
	GPIO_Set(GPIOA, PIN9 | PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_PU); //PA10,PA9,���ù���,�������
	GPIO_AF_Set(GPIOA, 10, 7);					//PA9, AF7
	GPIO_AF_Set(GPIOA, 9, 7);  					//PA10,AF7
	//����������
	USART1->BRR = mantissa; 	    			//����������
	USART1->CR1 &= ~(1 << 15); 	    			//����OVER8=0
	USART1->CR3 |= USART_CR3_DMAT;       		//����DMAʹ��
	USART1->CR1 |= USART_CR1_TE;  	     		//���ڷ���ʹ��
	mantissa = USART1->DR ;              		//���һ��DR�Ĵ���
	USART1->CR3 |= USART_CR3_DMAR;       		//����DMAʹ��
	USART1->CR1 |= USART_CR1_RE;  	     		//���ڽ���ʹ��
	USART1->CR1 |= USART_CR1_UE;  	     		//����ʹ��
}

////////////////////
/// ����״̬��
////////////////////
#define UART_IDLE 0
#define UART_ST1  1
#define UART_ST2  2
#define UART_ST3  3
#define UART_ST4  4
#define UART_ST5  5
#define UART_ST6  6
#define UART_ST7  7
#define UART_OVE  8

extern SYS_GSV		G_StaV_St ;

u32 UART1_GPS_OveTimeS ;        //��ʱ����������
u8  UART1_GPS_OveTime50MS;     	//

//����У����㣬Э��ֻ�����'$'��BCC�ֽ�֮������д������ֽ����
u8 CheckBccGPS(u8 * CheckBuf, u16 CheckLen)
{
	u16 i;
	u8  TempBcc = 0 ;

	for(i = 0; i < CheckLen; i++)
		TempBcc = TempBcc ^ CheckBuf[i];

	return TempBcc ;
}

//////////////////////////////////////////
//// ����ѭ�������������ݽ���Ϊ��Ч֡ѭ��
//// ���ǽ��ճ�����Ϊ���ӵĺ�����
//// ���г�ʱ����
u16 Proc_RxByte_GPS(void)
{
	static u8  Uart1_GPS_STA  = UART_IDLE ;
	static u16 RxProPos       = 0 ;

	static u8  UART1_GPS_OveTime_Flag = 1 ;										//��ʱ��־λ 1��ʱ����У�0������ʱ�ж�

	u16 i = 0 ;
	u8  ReaD ;
	u8  BCCtemp1, BCCtemp2;

	//���ڳ�ʱ�����յ���$����ʼ��ʱ������1s��Ϊ��ʱ���ָ�������״̬
	if( UART1_GPS_OveTime_Flag != 1 )
	{
		if( CheckTimerOve(UART1_GPS_OveTimeS, UART1_GPS_OveTime50MS) == 1
				&&  Uart1_GPS_STA != UART_IDLE )
		{
			Uart1_GPS_STA = UART_IDLE ;
			UART1_GPS_OveTime_Flag = 1;
		}
	}

	//ȡ�����ڽ��յ��ĸ��ֽ�λ��
	//U1_GPS_Rx_GetPosʼ��ָ��һ��DMA��û���õ�λ�ã�������
	//U1_GPS_Rx_ProcPosʼ��ָ��һ����û�д����λ��
	//��������һ����˵����û����Ҫ������ֽڣ�����û�п���ͻȻ���˺ܶ��ֽ�
	//ѭ������Ĵ���
	//ֻ����Ϊ���ջ������㹻�󣬴󵽲����ܳ���ѭ���������

	U1_GPS_Rx_GetPos = ( UARTRXBUFLEN_U1 - DMA2_Stream2->NDTR ) ;

	//��ദ��20���ֽڣ�����ʱ��̫����Ӱ����ѭ��
	while(i < 20)
	{
		if(U1_GPS_Rx_ProcPos == U1_GPS_Rx_GetPos)     							//û��Ҫ��������
			break;
		else
		{
			ReaD = U1_GPS_RxBuf[U1_GPS_Rx_ProcPos] ;    						//��Ҫ������ֽڣ�ȡ��

			U1_GPS_Rx_ProcPos++;                        						//ȡ����ѭ��+1
			U1_GPS_Rx_ProcPos %= UARTRXBUFLEN_U1;

			switch(Uart1_GPS_STA)
			{
				case UART_IDLE:                           						//����״̬��ֻ���յ�$���Ż�����

					if(ReaD == '$')
					{
						RxProPos = 0 ;
						U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						UART1_GPS_OveTime_Flag = 0 ;                			//��ʼ��ʱ����

						UART1_GPS_OveTimeS     = G_StaV_St.RunTime + 1 ;        	//���ó�ʱʱ��1s
						UART1_GPS_OveTime50MS = G_StaV_St.RunTime_50ms;    		//

						Uart1_GPS_STA = UART_ST1 ;
					}
					else
					{
						UART1_GPS_OveTime_Flag = 1 ;
						Uart1_GPS_STA = UART_IDLE ;
					}

					break;

				case UART_ST1:

					U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos] = ReaD ;
					RxProPos++;

					if( RxProPos >= 6 )
					{
						if((U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][3] == 'R'
								&& U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][4] == 'M'
								&& U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][5] == 'C')
								|| ( U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][3] == 'G'
									 && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][4] == 'S'
									 && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][5] == 'V')) /*)*/
							Uart1_GPS_STA = UART_ST2;
						else
						{
							UART1_GPS_OveTime_Flag = 1 ;
							Uart1_GPS_STA = UART_IDLE ;
						}
					}

					break;

				case UART_ST2:

					U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos] = ReaD ;
					RxProPos++;

					if(ReaD == '*')
					{
						Uart1_GPS_STA = UART_ST3;
					}

					if(RxProPos >= 500)
					{
						UART1_GPS_OveTime_Flag = 1 ;
						Uart1_GPS_STA = UART_IDLE ;
					}

					break;

				case UART_ST3:
					U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos] = ReaD ;
					RxProPos++;
					Uart1_GPS_STA = UART_ST4;
					break;

				case UART_ST4:

					U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos] = ReaD ;
					RxProPos++;

					if(((U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] >= '0' && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] <= '9')
							||  (U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] >= 'A' && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] <= 'F'))
							&& ((U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] >= '0' && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] <= '9')
								||  (U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] >= 'A' && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] <= 'F'))
					  )
					{
						if(U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] >= '0' && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] <= '9')
							BCCtemp1 = U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] - '0';
						else
							BCCtemp1 = U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 2] - 'A' + 10;

						if(U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] >= '0' && U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] <= '9')
							BCCtemp2 = U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] - '0';
						else
							BCCtemp2 = U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos - 1] - 'A' + 10;
					}
					else
					{
						UART1_GPS_OveTime_Flag = 1 ;
						Uart1_GPS_STA = UART_IDLE ;
					}

					BCCtemp1 = (BCCtemp1 << 4) + BCCtemp2 ;

					if(BCCtemp1 ==  CheckBccGPS(&(U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][1]), RxProPos - 4))
					{
						U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						//������ѱ��ĵĳ�������ˡ�$��
						U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][0] = RxProPos ;
						//���������Ч֡
						U1_GPS_Rx_GetFrame++;
						U1_GPS_Rx_GetFrame %= 20   ;
					}

					UART1_GPS_OveTime_Flag = 1 ;
					Uart1_GPS_STA = UART_IDLE ;
					break;

				case UART_OVE:
					break;

				default:
					break;
			}
		}

		i++;
	}

	U1_GPS_Rx_GetPos = ( UARTRXBUFLEN_U1 - DMA2_Stream2->NDTR ) ;
	//���ػ��ж��ٸ��ֽ�û�д���
	return (U1_GPS_Rx_GetPos + UARTRXBUFLEN_U1 - U1_GPS_Rx_ProcPos) % UARTRXBUFLEN_U1 ;
}
//

//////////////////////////////////////////////////////
///  ��������ȡ��һ�����ϵ���Ч֡���������
///
//////////////////////////////////////////////////////
u8 Proc_RxCMD_GPS(u8 * DataBuf)
{
	u16 i;
	u16 DataLen;
	u8  CMD_Temp ;

	if( U1_GPS_Rx_GetFrame != U1_GPS_Rx_ProcFrame )
	{
		DataLen = U1_GPS_RxFrame[U1_GPS_Rx_ProcFrame][0];

		for(i = 0; i < DataLen; i++)
			DataBuf[i] = U1_GPS_RxFrame[U1_GPS_Rx_ProcFrame][i] ;

		CMD_Temp = U1_GPS_RxFrame[U1_GPS_Rx_ProcFrame][5] ;

		U1_GPS_Rx_ProcFrame++;
		U1_GPS_Rx_ProcFrame %= 20 ;

		return CMD_Temp ;
	}

	return 0 ;
}
//ȡ�����򣬲�Ҫ'.'
u8 Get_Region_Pos(u8* StrIn, u8 Num, u8 * OutStr)
{
	u8 TempPos;
	u8 i ;

	for(TempPos = 0; TempPos < 200; TempPos++)
	{
		if(Num == 0)
			break;

		if(StrIn[TempPos] == ',')
			Num-- ;
	}

	//TempPosָ����Ҫȡ�����ĵ�һ���ֽ�
	if( TempPos >= 200)
		return 0xFF ;//���ش���

	for(i = 0; i < 50; i++)
	{
		if(StrIn[TempPos + i] == ',' || StrIn[TempPos + i] == '*' || StrIn[TempPos + i] == '.')
			break;

		OutStr[i] = StrIn[TempPos + i];
	}

	if(i >= 50)
		return 0xFF ;//���ش���

	OutStr[i] = 0 ;//�ַ�����NULL����

	return i;
}
//ȡ������Ҫ'.'
u8 Get_Region_Pos_wp(u8* StrIn, u8 Num, u8 * OutStr)
{
	u8 TempPos;
	u8 i ;

	for(TempPos = 0; TempPos < 200; TempPos++)
	{
		if(Num == 0)
			break;

		if(StrIn[TempPos] == ',')
			Num-- ;
	}

	//TempPosָ����Ҫȡ�����ĵ�һ���ֽ�
	if( TempPos >= 200)
		return 0xFF ;//���ش���

	for(i = 0; i < 50; i++)
	{
		if(StrIn[TempPos + i] == ',' || StrIn[TempPos + i] == '*')
			break;

		OutStr[i] = StrIn[TempPos + i];
	}

	if(i >= 50)
		return 0xFF ;//���ش���

	OutStr[i] = 0 ;//�ַ�����NULL����

	return i;
}
//
//����RMC��GSV����
void Prorc_RNSS_GPS_RMC_GSV( u8 * RNSS_GPSData )
{
	u8 	Region_Str[50] ;
	u8  Sec, Min, Hor;
	u8  Mon, Dat;
	u16 YearQ;

	double tempVal;

	RTC_TIME_Str RTC_TIME_Var ;

	if(Get_Region_Pos(RNSS_GPSData, 0, Region_Str) != 6)
		return;

	if(strcmp("$GPRMC", (char *)Region_Str) == 0 || strcmp("$GNRMC", (char *)Region_Str) == 0)
	{
		if(Get_Region_Pos(RNSS_GPSData, 2, Region_Str) != 1)
			return;

		if(Region_Str[0] == 'A')                  						//�յ���λ��Ч����
		{
			if(Get_Region_Pos(RNSS_GPSData, 1, Region_Str) != 6)
				return;

			Hor = (Region_Str[0] - '0') * 10 + (Region_Str[1] - '0') ;
			Min = (Region_Str[2] - '0') * 10 + (Region_Str[3] - '0') ;
			Sec = (Region_Str[4] - '0') * 10 + (Region_Str[5] - '0') ;

			if(Get_Region_Pos(RNSS_GPSData, 9, Region_Str) != 6)
				return;

			Dat = (Region_Str[0] - '0') * 10 + (Region_Str[1] - '0') ;
			Mon = (Region_Str[2] - '0') * 10 + (Region_Str[3] - '0') ;
			YearQ = (Region_Str[4] - '0') * 10 + (Region_Str[5] - '0') ;
			YearQ = 2000 + YearQ;

			RTC_TIME_Var.Sec = Sec;
			RTC_TIME_Var.Min = Min;
			RTC_TIME_Var.Hor = Hor;
			RTC_TIME_Var.Dat = Dat;
			RTC_TIME_Var.Mon = Mon;
			RTC_TIME_Var.Yea = (u8)(YearQ - 2000);
			RTC_TIME_Var.QYear = YearQ ;

			Rtc2UnixT(&RTC_TIME_Var);

			G_StaV_St.GPS_Time = RTC_TIME_Var.Unix_Time + 28800;		//���������28800,����8��Сʱ,���㵽����ʱ��

			if( G_StaV_St.GPS_STA != GPS_RNSS_A )                		//����֮ǰ����֮ǰ��״̬�ǲ�����Ч
			{
				//���������Ч,����PC������һ��״̬
				G_StaV_St.GPS_STA  = GPS_RNSS_A;
				Send_STA_Sys();
			}

			//ȡ����γ������

			if(	Get_Region_Pos_wp(RNSS_GPSData, 3, Region_Str) == 9)			//ȡ��γ������	GPS15LЭ�飬С�����4λ
			{
				G_StaV_St.GPS_latitude = (Region_Str[0] - '0') * 10.0f + (Region_Str[1] - '0');

				tempVal = (Region_Str[2] - '0') * 10.0f
						  + (Region_Str[3] - '0') * 1.0f
						  + (Region_Str[5] - '0') * 0.1f
						  + (Region_Str[6] - '0') * 0.01f
						  + (Region_Str[7] - '0') * 0.001f
						  + (Region_Str[8] - '0') * 0.0001f ;

				G_StaV_St.GPS_latitude += tempVal / 60.0f;
			}
			else if( Get_Region_Pos_wp(RNSS_GPSData, 3, Region_Str) == 10)		//ȡ��γ������ UBloxЭ�飬С�����5λ
			{
				G_StaV_St.GPS_latitude = (Region_Str[0] - '0') * 10.0f + (Region_Str[1] - '0');

				tempVal = (Region_Str[2] - '0') * 10.0f
						  + (Region_Str[3] - '0') * 1.0f
						  + (Region_Str[5] - '0') * 0.1f
						  + (Region_Str[6] - '0') * 0.01f
						  + (Region_Str[7] - '0') * 0.001f
						  + (Region_Str[8] - '0') * 0.0001f
						  + (Region_Str[9] - '0') * 0.00001f ;

				G_StaV_St.GPS_latitude += tempVal / 60.0f;
			}

			Get_Region_Pos(RNSS_GPSData, 4, Region_Str);

			if(Region_Str[0] == 'N')
				G_StaV_St.GPS_latitude =  G_StaV_St.GPS_latitude ;				//��γ+
			else
				G_StaV_St.GPS_latitude = -G_StaV_St.GPS_latitude;				//��γ-

			if(	Get_Region_Pos_wp(RNSS_GPSData, 5, Region_Str) == 10)			//ȡ����������	GPS15LЭ�飬С�����4λ
			{
				G_StaV_St.GPS_longitude = (Region_Str[0] - '0') * 100.0f + (Region_Str[1] - '0') * 10.0f + (Region_Str[2] - '0');

				tempVal = (Region_Str[3] - '0') * 10.0f
						  + (Region_Str[4] - '0') * 1.0f
						  + (Region_Str[6] - '0') * 0.1f
						  + (Region_Str[7] - '0') * 0.01f
						  + (Region_Str[8] - '0') * 0.001f
						  + (Region_Str[9] - '0') * 0.0001f ;

				G_StaV_St.GPS_longitude += tempVal / 60.0f;
			}
			else if( Get_Region_Pos_wp(RNSS_GPSData, 5, Region_Str) == 11)	//ȡ����������	UBloxЭ�飬С�����5λ
			{
				G_StaV_St.GPS_longitude = (Region_Str[0] - '0') * 100.0f + (Region_Str[1] - '0') * 10.0f + (Region_Str[2] - '0');

				tempVal = (Region_Str[3] - '0' ) * 10.0f
						  + (Region_Str[4] - '0' ) * 1.0f
						  + (Region_Str[6] - '0' ) * 0.1f
						  + (Region_Str[7] - '0' ) * 0.01f
						  + (Region_Str[8] - '0' ) * 0.001f
						  + (Region_Str[9] - '0' ) * 0.0001f
						  + (Region_Str[10] - '0') * 0.00001f ;

				G_StaV_St.GPS_longitude += tempVal / 60.0f;
			}

			Get_Region_Pos(RNSS_GPSData, 6, Region_Str);

			if(Region_Str[0] == 'E')
				G_StaV_St.GPS_longitude =  G_StaV_St.GPS_longitude;	//����+
			else
				G_StaV_St.GPS_longitude = -G_StaV_St.GPS_longitude;	//����-

			if(G_StaV_St.Messa_1PPS == 1)
				GPSD_Send_Flag	= 1 ;	//������GPS���ĸ���λ��
			else
				GPSD_Send_Flag	= 0 ;

			GPS_A_V_TimerOve      = G_StaV_St.RunTime + 2;    		//����GPS���ĸ��³�ʱ,�������֮��
			GPS_A_V_TimerOve50ms  = G_StaV_St.RunTime_50ms;   		//û�и���,�Ͱ�GPS����Ϊ��Ч.

		}
		else                                      					//�յ�'V'����
		{
			if(G_StaV_St.GPS_STA != GPS_RNSS_V ) 					//PotA �����ǹػ�������Ч,����û���ź���.
			{
				//����GPS״̬,�����ϱ�PC��
				G_StaV_St.GPS_STA = GPS_RNSS_V;
				Send_STA_Sys();
			}
		}
	}

	//����GSV����,��ʵֻ�ѿɼ�����ȡ����
	if(strcmp("$GPGSV", (char *)Region_Str) == 0 )
	{
		if(Get_Region_Pos(RNSS_GPSData, 3, Region_Str) != 2)
			return;

		G_StaV_St.GPS_SatN = (Region_Str[0] - '0') * 10 + (Region_Str[1] - '0') ;
	}

}
//////////////////////////////
////GPS���ĵĶ��㴦����
////
void Prorc_GPS_Top(void)
{
	u8	New_GPS_Flag = 0;				//�µ�GPS�������GPS
	u8	New_GPS_Data[512];				//����GPS�ı���
	u8	GPSLenTemp;						//GPS���ĵĳ���

	//����GPS����
	if( G_StaV_St.GPS_STA != GPS_RNSS_OFF )											//���ﴦ��GPS��صı��ģ����GPSʱ�ػ��ģ��Ͳ�������
	{
		Proc_RxByte_GPS();															//����GPSԭʼ����
		New_GPS_Flag 	= Proc_RxCMD_GPS(New_GPS_Data);								//ȡ��һ�����ϵı���

		if( New_GPS_Flag	!=	0 )													//������µ�GPS���ĵ���
		{
			GPSLenTemp 		= 	New_GPS_Data[0] ;
			New_GPS_Data[0]	=	'$' ;
			Prorc_RNSS_GPS_RMC_GSV( New_GPS_Data );									//����GPS���ģ�����״̬��ʱ�䣬�����ȵ���Ϣ
			New_GPS_Data[0] = GPSLenTemp ;

			if( G_StaV_St.Messa_GPS == 1 )											//�����GPSת��ʹ��
			{
				if(New_GPS_Flag == 'C')												//RMCת��ȫ��
				{
					//ת��GPS����
					Send_GPS_C_V(New_GPS_Data);
				}
				else if( New_GPS_Flag == 'V' && New_GPS_Data[9] == '1')				//GSVֻת����1��
				{
					//ת��GPS����
					//Send_GPS_C_V(New_GPS_Data);									//����ת��GSV����
				}
			}

			New_GPS_Flag = 0 ;
		}
	}
}

//GPS��Դ���Ƴ�ʼ��
void GPS_Pow_init(void)
{
	RCC->AHB1ENR    |=  RCC_AHB1ENR_GPIOCEN;//ʹ��PORTCʱ��
	POWGPS = 0 ;
	GPIO_Set(GPIOC, PIN5, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); //

	RCC->AHB1ENR    |=  RCC_AHB1ENR_GPIOCEN;//ʹ��PORTCʱ��  GPS_P_C
	GPIO_Set(GPIOC, PIN9, GPIO_MODE_IN, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE);  //

	G_StaV_St.GPS_STA 	=	GPS_RNSS_OFF ;
	GPS_Pow(0);
}

//����GPS��Դ
void GPS_Pow(u8 OnOff)
{
	if(OnOff)     POWGPS = 1 ;
	else          POWGPS = 0 ;
}
///////////////////////////////////////////////////////////
///���Ǹ��º���,�����Ǽ���ⲿGPS��Դ����IO,ˢ��GPSģ��ĵ�Դ
void GPS_Pow_Re(void)
{
	static u8  	GPS_Power = 0 ;		//��ǰ��GPS��Դ�Ƿ��
	static u16	GPS_V_Timer = 0;	//GPS��Դ�������,������Ч״̬��ʱ��,�����Զ�����GPS

	u8	GPSPow_PinV;

	GPSPow_PinV	= GPS_P_C ; 		//�õ��ⲿ�����ź� 1���ر� 0����

	if( GPSPow_PinV == 0)			//�ⲿҪ���GPS
	{
		if(GPS_Power == 1)			//��������Ѿ���
		{
		}
		else						//����û�д�
		{
			G_StaV_St.GPS_STA 	=	GPS_RNSS_V ;
			GPS_Power 			= 1;
			GPS_Pow(GPS_Power);
			GPS_V_Timer 		= 	0 ;
			Send_STA_Sys();
		}
	}
	else							//�ⲿҪ��رյ�Դ
	{
		if(GPS_Power == 1)			//��������Ѿ���
		{
			G_StaV_St.GPS_STA 	=	GPS_RNSS_OFF ;
			GPS_Power 			= 0;
			GPS_Pow(GPS_Power);
			GPS_V_Timer 		= 	0 ;
			Send_STA_Sys();
		}
		else						//����û�д�
		{
		}
	}

	//���´��룬���GPS�ĸ�λ����GPS��600sû�ж�λ�󣬹ص�GPS��Դ��3s�󣬴�GPS��Դ
	if(G_StaV_St.GPS_STA  == GPS_RNSS_V)
	{
		GPS_V_Timer++;				//ֻ����GPS��Ч���״̬�£���ȥ����ʱ��

		if(GPS_V_Timer >= 600 && GPS_V_Timer < 800)
		{
			GPS_V_Timer = 800 ;		//����800������������״̬������û��ȡ��GPS_Power����Ϊ��ǰ��GPS���ڿ���״̬
			GPS_Pow(0);				//�ر�GPS��Դ
		}

		if(GPS_V_Timer > 802)
		{
			GPS_V_Timer = 0 ;		//�������´�GPS��Դ 801��802��803���ϼ�3s�����м�����ر�gps���ͻ�ȥ����״̬����ȫ��
			GPS_Pow(1);				//��GPS��Դ
		}
	}
	else
	{
		GPS_V_Timer = 0 ;			//��Ч���ػ������ᵽ�������GPS_V_Timer
	}
}


