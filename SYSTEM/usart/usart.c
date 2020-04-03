#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "DIGPOT.h"
#include "GPS.h"
#include "stmflash.h"
#include "24cxx.h"

extern SYS_GSV		G_StaV_St ;

//���ڽ������ݻ�������DMA�Զ���䣬ѭ������
#include <stm32f4xx.h>
__align(4) u8  U6_Sys_RxBuf[UARTRXBUFLEN] ;
u16 U6_Sys_Rx_ProcPos = 0;          //ָ��û�д�����ֽ�
u16 U6_Sys_Rx_GetPos  = 0;          //ָ��û�б������ֽڣ��ڽ��մ������и���

//������������Ч֡
__align(4) u8  U6_Sys_RxFrame[8][512] ;
u16 U6_Sys_Rx_ProcFrame = 0;        //ָ��û�д������Ч֡
u16 U6_Sys_Rx_GetFrame  = 0;        //ָ��û�б����������Ŀ�֡

//�����֡
__align(4) u8  U6_Sys_TxFrame[20][512] ;
u16 U6_Sys_Tx_InFrame = 0;          //ָ��û�б�������������֡����֡
u16 U6_Sys_Tx_OuFrame = 0;          //ָ��û�з������֡

//////////////////////////////////////
////  UART6 RX ʹ�õ�DMA
////  DMA2 Stream1 Ch5 ��ʼ��
void UART6_RX_DMA_Config(void)
{
	DMA_Stream_TypeDef * DMA_Streamx ;
	DMA_TypeDef *DMAx;
	u8 streamx;
	u8 chx ;

	DMA_Streamx = DMA2_Stream1 ;
	DMAx        = DMA2 ;
	streamx     = 1 ;
	chx         = 5 ;

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; //DMA2ʱ��ʹ��

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 	//�ر�DMA����

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

	DMA_Streamx->PAR  = (u32) & (USART6->DR);		 		//DMA�����ַ
	DMA_Streamx->M0AR = (u32)U6_Sys_RxBuf ;		 		//DMA �洢��0��ַ
	DMA_Streamx->NDTR = UARTRXBUFLEN;		       		//
	DMA_Streamx->CR   = 0;			                	//��ȫ����λCR�Ĵ���ֵ

	DMA_Streamx->CR  &= ~DMA_SxCR_DIR;		   //���赽�洢��ģʽ
	DMA_Streamx->CR  |= DMA_SxCR_CIRC;	    	//ѭ��ģʽ
	DMA_Streamx->CR  &= ~DMA_SxCR_PINC;		  	//���������ģʽ
	DMA_Streamx->CR  |= DMA_SxCR_MINC;		  	//�洢������ģʽ
	DMA_Streamx->CR  &= ~DMA_SxCR_PSIZE;		  	//�������ݳ���:8λ
	DMA_Streamx->CR  &= ~DMA_SxCR_MSIZE;		  	//�洢�����ݳ���:8λ
	DMA_Streamx->CR  &= ~DMA_SxCR_PL;		   //�����ȼ�
	DMA_Streamx->CR  &= ~DMA_SxCR_PBURST;		//����ͻ�����δ���
	DMA_Streamx->CR  &= ~DMA_SxCR_MBURST;		//�洢��ͻ�����δ���

	DMA_Streamx->CR  |= (u32)chx << 25;        	//ͨ��ѡ��

	DMA_Streamx->CR  &= ~DMA_SxCR_TCIE ;     	//��Ҫ�ж�

	DMA_Streamx->CR  |= DMA_SxCR_EN;        	//��DMA
}
//////////////////////////////////////
////  UART6 TX ʹ�õ�DMA
////  DMA2 Stream6 Ch5 ��ʼ��
void UART6_TX_DMA_Config(void)
{
	DMA_Stream_TypeDef * DMA_Streamx ;
	DMA_TypeDef *DMAx;
	u8 streamx;
	u8 chx ;

	DMA_Streamx = DMA2_Stream6 ;
	DMAx        = DMA2 ;
	streamx     = 6 ;
	chx         = 5 ;

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;               //DMA2ʱ��ʹ��

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 	              //�ر�DMA����

	while(1)
	{
		if( (DMA_Streamx->CR & DMA_SxCR_EN) == 0 )
			break;
	}

	streamx = (((u32)DMA_Streamx - (u32)DMAx) - 0X10) / 0X18;	//�õ�streamͨ����

	if(streamx >= 6)
		DMAx->HIFCR |= 0X3D << (6 * (streamx - 6) + 16);	      		//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx >= 4)
		DMAx->HIFCR |= 0X3D << 6 * (streamx - 4);                 	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx >= 2)
		DMAx->LIFCR |= 0X3D << (6 * (streamx - 2) + 16);            	//���֮ǰ��stream�ϵ������жϱ�־
	else
		DMAx->LIFCR |= 0X3D << 6 * streamx;						//���֮ǰ��stream�ϵ������жϱ�־

	DMA_Streamx->PAR  = (u32) & (USART6->DR);		  		//DMA�����ַ
	DMA_Streamx->M0AR = (u32)0 ;		           		//DMA �洢��0��ַ
	DMA_Streamx->NDTR = 100;		              		//
	DMA_Streamx->CR   = 0;			            		//��ȫ����λCR�Ĵ���ֵ

	DMA_Streamx->CR  |= DMA_SxCR_DIR_0;		  			//�洢��������ģʽ
	DMA_Streamx->CR  &= ~DMA_SxCR_CIRC;					//��ѭ��ģʽ
	DMA_Streamx->CR  &= ~DMA_SxCR_PINC;					//���������ģʽ
	DMA_Streamx->CR  |= DMA_SxCR_MINC;					//�洢������ģʽ
	DMA_Streamx->CR  &= ~DMA_SxCR_PSIZE;		  			//�������ݳ���:8λ
	DMA_Streamx->CR  &= ~DMA_SxCR_MSIZE;		  			//�洢�����ݳ���:8λ
	DMA_Streamx->CR  &= ~DMA_SxCR_PL;		    		//�����ȼ�
	DMA_Streamx->CR  &= ~DMA_SxCR_PBURST;				//����ͻ�����δ���
	DMA_Streamx->CR  &= ~DMA_SxCR_MBURST;				//�洢��ͻ�����δ���

	DMA_Streamx->CR  |= (u32)chx << 25;          			//ͨ��ѡ��
	DMA_Streamx->CR  &= ~DMA_SxCR_TCIE ;       			//���ж�
	DMA_Streamx->CR  &= ~DMA_SxCR_EN;          			//���������ȴ����͵�ʱ��������
}

//���÷���1֡����������Ӧ��DMA������
void Enable_U6_Tx_DMA(u8 * Tx_Buf, u16 Tx_Num)
{
	DMA_Stream_TypeDef * DMA_Streamx ;
	DMA_TypeDef *DMAx;
	u8 streamx;

	DMA_Streamx = DMA2_Stream6 ;
	DMAx        = DMA2 ;
	streamx     = 6 ;

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 	//�ر�DMA����

	while(1)
	{
		if( (DMA_Streamx->CR & DMA_SxCR_EN) == 0 )
			break;
	}

	streamx = (((u32)DMA_Streamx - (u32)DMAx) - 0X10) / 0X18;	//�õ�streamͨ����

	if(streamx >= 6)
		DMAx->HIFCR |= 0X3D << (6 * (streamx - 6) + 16);	          	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx >= 4)
		DMAx->HIFCR |= 0X3D << 6 * (streamx - 4);                 	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx >= 2)
		DMAx->LIFCR |= 0X3D << (6 * (streamx - 2) + 16);            	//���֮ǰ��stream�ϵ������жϱ�־
	else
		DMAx->LIFCR |= 0X3D << 6 * streamx;						//���֮ǰ��stream�ϵ������жϱ�־

	DMA_Streamx->M0AR = (u32)Tx_Buf ;
	DMA_Streamx->NDTR = Tx_Num ;

	DMA_Streamx->CR |= DMA_SxCR_EN;                     //�������ݴ��䣬������ϣ��Զ�ֹͣ����ѭ��ģʽ
}

//////////////////////////////////////////////////////////////////////////////////
//// UART6��SYS��Ƭ���Ĵ��ڳ�ʼ��
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
void UART6_SYS_init(u32 pclk2, u32 bound)
{
	float temp;

	u16 mantissa;
	u16 fraction;

	UART6_RX_DMA_Config();                  //����ѭ������DMA��������

	UART6_TX_DMA_Config();                  //���÷���DMA������������ֻ������DMAģʽ

	temp = (float)(pclk2 * 1000000) / (bound * 16); //�õ�USARTDIV@OVER8=0
	mantissa = temp;				                //�õ���������
	fraction = (temp - mantissa) * 16;      //�õ�С������@OVER8=0
	mantissa <<= 4;
	mantissa += fraction;

	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN ;   	//ʹ��PORTC��ʱ��
	RCC->APB2ENR  |= RCC_APB2ENR_USART6EN;  	//ʹ�ܴ���6ʱ��

	RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;    //��λ����6
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;

	GPIO_Set(GPIOC, PIN6 | PIN7, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE); //PC6,PC7,���ù���,�������
	GPIO_AF_Set(GPIOC, 6, 8);	//PC6,AF8
	GPIO_AF_Set(GPIOC, 7, 8); //PC7,AF8

	//����������
	USART6->BRR = mantissa; 	            //����������
	USART6->CR1 &= ~(1 << 15); 	         //����OVER8=0

	USART6->CR3 |= USART_CR3_DMAT;          //����DMAʹ��
	USART6->CR1 |= USART_CR1_TE;  	        //���ڷ���ʹ��

	mantissa = USART6->DR ;                 //���һ��DR�Ĵ���

	USART6->CR3 |= USART_CR3_DMAR;          //����DMAʹ��
	USART6->CR1 |= USART_CR1_RE;  	        //���ڽ���ʹ��

	USART6->CR1 |= USART_CR1_UE;  	        //����ʹ��
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

////////////////////
/// ����ͨ�ŵ�ַ
////////////////////
#define SIGUADD 'S'   //�źŵ�Ƭ��
#define TIMUADD 'T'   //��ʱ��Ƭ��
#define SYSUADD 'M'   //���ص�Ƭ��
#define PCZUADD 'P'   //��λ��

u32 UART6_SYS_OveTimeS ;        //��ʱ����������
u8  UART6_SYS_OveTime50MS; 		//

//����У����㣬Э��ֻ�����'$'��BCC�ֽ�֮������д������ֽ����
u8 CheckBcc(u8 * CheckBuf, u16 CheckLen)
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
u16 Proc_RxByte_SYS(void)
{
	static u8  Uart6_Sys_STA  = UART_IDLE ;
	static u16 RxProPos       = 0 ;
	static u16 DataLen        = 0 ;

	static u8  UART6_SYS_OveTime_Flag = 1 ;//��ʱ��־λ 1��ʱ����У�0������ʱ�ж�

	u16 i = 0 ;
	u8  ReaD ;

	//���ڳ�ʱ�����յ���$����ʼ��ʱ������1s��Ϊ��ʱ���ָ�������״̬
	if( UART6_SYS_OveTime_Flag != 1 )
	{
		if( CheckTimerOve(UART6_SYS_OveTimeS, UART6_SYS_OveTime50MS) == 1
				&&  Uart6_Sys_STA != UART_IDLE )
		{
			Uart6_Sys_STA = UART_IDLE ;
			UART6_SYS_OveTime_Flag = 1;
		}
	}

	//ȡ�����ڽ��յ��ĸ��ֽ�λ��
	//U6_Sys_Rx_GetPosʼ��ָ��һ��DMA��û���õ�λ�ã�������
	//U6_Sys_Rx_ProcPosʼ��ָ��һ����û�д����λ��
	//��������һ����˵����û����Ҫ������ֽڣ�����û�п���ͻȻ���˺ܶ��ֽ�
	//ѭ������Ĵ���
	//ֻ����Ϊ���ջ������㹻�󣬴󵽲����ܳ���ѭ���������

	U6_Sys_Rx_GetPos = ( UARTRXBUFLEN - DMA2_Stream1->NDTR ) ;

	//��ദ��10���ֽڣ�����ʱ��̫����Ӱ����ѭ��
	while(i < 10)
	{
		if(U6_Sys_Rx_ProcPos == U6_Sys_Rx_GetPos)     	//û��Ҫ��������
			break;
		else
		{
			ReaD = U6_Sys_RxBuf[U6_Sys_Rx_ProcPos] ;    //��Ҫ������ֽڣ�ȡ��

			U6_Sys_Rx_ProcPos++;                        //ȡ����ѭ��+1
			U6_Sys_Rx_ProcPos %= UARTRXBUFLEN;

			switch(Uart6_Sys_STA)
			{
				case UART_IDLE:                           //����״̬��ֻ���յ�$���Ż�����

					if(ReaD == '$')
					{
						RxProPos = 0 ;
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						UART6_SYS_OveTime_Flag = 0 ;                //��ʼ��ʱ����

						UART6_SYS_OveTimeS   	= 	G_StaV_St.RunTime + 1 ;        	//���ó�ʱʱ��1s
						UART6_SYS_OveTime50MS 	=	G_StaV_St.RunTime_50ms;    		//

						Uart6_Sys_STA = UART_ST1 ;
					}
					else
					{
						UART6_SYS_OveTime_Flag = 1 ;
						Uart6_Sys_STA = UART_IDLE ;
					}

					break;

				case UART_ST1:
					if(ReaD == TIMUADD)                    		//Ŀ�ĵ�ַ�жϣ��Ƿ����źŵ�Ƭ��
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						Uart6_Sys_STA = UART_ST2;
					}
					else
					{
						UART6_SYS_OveTime_Flag = 1 ;
						Uart6_Sys_STA = UART_IDLE ;
					}

					break;

				case UART_ST2:                            		//Դ��ַ�жϣ��Ƿ������ص�Ƭ��
					if(ReaD == PCZUADD)
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						Uart6_Sys_STA = UART_ST3;
					}
					else
					{
						UART6_SYS_OveTime_Flag = 1 ;
						Uart6_Sys_STA = UART_IDLE ;
					}

					break;

				case UART_ST3:                                //�������Ƿ�����Ч����
					if(ReaD <= 'Z' && ReaD >= 'A')
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						Uart6_Sys_STA = UART_ST4;
					}
					else
					{
						UART6_SYS_OveTime_Flag = 1 ;
						Uart6_Sys_STA = UART_IDLE ;
					}

					break;

				case UART_ST4:                                		//�����򳤶��Ƿ���ȷ
					if(ReaD <= 250)                             	//0-200�������򳤶�Ϊ0-200
					{
						//201Ϊ 200+(201-200)*2 = 202
						if(ReaD <= 200)                           	//250Ϊ 200+(250-200)*2 = 300
							DataLen = ReaD ;                        //����250���ж�Ϊ����
						else
							DataLen = 2 * (ReaD - 200) + 200 ;

						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						if( DataLen == 0 )                        	//����������򳤶�Ϊ0��ֱ������BCCУ��
							Uart6_Sys_STA = UART_ST6 ;          	//�����������������
						else
							Uart6_Sys_STA = UART_ST5 ;
					}
					else
					{
						UART6_SYS_OveTime_Flag = 1 ;
						Uart6_Sys_STA = UART_IDLE ;
					}

					break;

				case UART_ST5:
					if( DataLen != 0 )                          //������ʵ�����жϣ�Ϊ�˺ÿ�����
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;
						DataLen--;
					}

					if( DataLen == 0 )                      	//��������������ϣ���ô����BCC���
						Uart6_Sys_STA = UART_ST6 ;
					else
						Uart6_Sys_STA = UART_ST5 ;

					break;

				case UART_ST6:                                	//BCC���
					if(ReaD == CheckBcc(&(U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][0]), RxProPos))
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						Uart6_Sys_STA = UART_ST7 ;
					}
					else
					{
						UART6_SYS_OveTime_Flag = 1 ;
						Uart6_Sys_STA = UART_IDLE ;
					}

					break;

				case UART_ST7:                                	//��֤�Ƿ���0x0A,����ɹ��յ�����

					if(ReaD == 0x0A)
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						//���������Ч֡
						U6_Sys_Rx_GetFrame++;
						U6_Sys_Rx_GetFrame %= 8;
					}

					UART6_SYS_OveTime_Flag = 1 ;
					Uart6_Sys_STA = UART_IDLE ;
					break;

				case UART_OVE:
					break;

				default:
					break;
			}
		}

		i++;
	}

	U6_Sys_Rx_GetPos = ( UARTRXBUFLEN - DMA2_Stream1->NDTR ) ;

	//���ػ��ж��ٸ��ֽ�û�д���
	return (U6_Sys_Rx_GetPos + UARTRXBUFLEN - U6_Sys_Rx_ProcPos) % UARTRXBUFLEN ;
}
//

//////////////////////////////////////////////////////
///      ��������֡
///
//////////////////////////////////////////////////////
void Proc_TxFrame_SYS(void)
{
	u8 FrameTxNum ;

	if(   U6_Sys_Tx_InFrame != U6_Sys_Tx_OuFrame    	//ȷ��������֡��Ҫ����
			&& (DMA2_Stream6->CR  &  DMA_SxCR_EN ) == 0 ) 		//ȷ��Ŀǰû���ڷ������ݣ�DMA�������Ѿ��Զ�ֹͣ�ˡ�
	{
		if( U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][0] == '$' //ȷ�ϳ�BCC�ֽ��⣬������������ȷ
				&& U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][1] == 'P'
				&& U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][2] == 'T'
				&& ((U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] <= 'Z' && U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] >= 'A') ||
					(U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] <= 'z' && U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] >= 'a'))
				&& U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] <= 250)
		{
			//����֡����
			if(U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] >= 200)
				FrameTxNum = (U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] - 200) * 2 + 250 + 7;
			else
				FrameTxNum = U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] + 7;

			if( U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][FrameTxNum - 1] == 0x0A )
			{
				U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][FrameTxNum - 2]
					= CheckBcc(&U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][0], FrameTxNum - 2);       		//����BCC

				Enable_U6_Tx_DMA( &U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][0], FrameTxNum );    	//����DMA����
			}
		}

		//�������֡��ʼ���ͣ������ѷ���ָ��
		//�������֡û�з��ͣ�ҲҪ���£�˵�������͵�����֡�д�������������
		U6_Sys_Tx_OuFrame++;
		U6_Sys_Tx_OuFrame %= 20;
	}
}
//////////////////////////////////////////////////////
///  ��������ȡ��һ�����ϵ���Ч֡���������
///
//////////////////////////////////////////////////////
u8 Proc_RxCMD_SYS(u8 * DataBuf)
{
	u16 i;
	u16 DataLen;
	u8  CMD_Temp ;

	if( U6_Sys_Rx_GetFrame != U6_Sys_Rx_ProcFrame )
	{

		if(U6_Sys_RxFrame[U6_Sys_Rx_ProcFrame][4] >= 200)
			DataLen = (U6_Sys_RxFrame[U6_Sys_Rx_ProcFrame][4] - 200) * 2 + 250 ;
		else
			DataLen = U6_Sys_RxFrame[U6_Sys_Rx_ProcFrame][4];

		//DataLen��󲻻ᳬ��300�ֽ�
		for(i = 0; i < DataLen; i++)
			DataBuf[i] = U6_Sys_RxFrame[U6_Sys_Rx_ProcFrame][i + 5];

		CMD_Temp = U6_Sys_RxFrame[U6_Sys_Rx_ProcFrame][3] ;

		U6_Sys_Rx_ProcFrame++;
		U6_Sys_Rx_ProcFrame %= 8 ;

		return CMD_Temp ;
	}

	return 0 ;
}
//

//////////////////////////////////////////////////////
///  ����TK״̬
///
//////////////////////////////////////////////////////
void Send_STA_Sys(void)
{
	UART_SysSTAVarSt * UART_SysSTAVar ;//__align(4)

	UART_SysSTAVar = ( UART_SysSTAVarSt * ) & (U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5]);
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 't' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] =  sizeof(UART_SysSTAVarSt)  ;

	UART_SysSTAVar->RunTime				=	G_StaV_St.RunTime;             	//����ʱ��
	UART_SysSTAVar->RunTime_50ms		=	G_StaV_St.RunTime_50ms;   		//50msΪ��λ������ʱ��
	UART_SysSTAVar->TimeKeepSTA			=	G_StaV_St.TimeKeepSTA;         	//��ѭ��״̬��
	UART_SysSTAVar->Temper				=	G_StaV_St.Temper;          		//��ǰоƬ�¶�
	UART_SysSTAVar->DigPot				=	G_StaV_St.DigPot;             	//���ֵ�λ��ֵ��Ĭ��Ϊ469
	UART_SysSTAVar->TKS_Flag			=	G_StaV_St.TKS_Flag;        		//��ʱ�ȶ���ʶ
	UART_SysSTAVar->GPS_STA				=	G_StaV_St.GPS_STA;         		//GPS״̬��ʾ
	UART_SysSTAVar->GPS_Time			=	G_StaV_St.GPS_Time;           	//GPSʱ��
	UART_SysSTAVar->GPS_SatN			=	G_StaV_St.GPS_SatN;            	//GPS�ɼ�����
	UART_SysSTAVar->GPS_longitude		=	(s32)(G_StaV_St.GPS_longitude * 10000000);     	//GPS����
	UART_SysSTAVar->GPS_latitude		=	(s32)(G_StaV_St.GPS_latitude * 10000000);      	//GPSγ��
	UART_SysSTAVar->NATI_Time			=	G_StaV_St.NATI_Time;			//����1PPSʱ��
	UART_SysSTAVar->NATI_Time_Prog		=	G_StaV_St.NATI_Time_Prog;		//�����ɱ������ʱ��
	UART_SysSTAVar->Last_SYN_Time		=	G_StaV_St.Last_SYN_Time;		//�ϴ�ͬ��ʱ��
	UART_SysSTAVar->Last_ACQ_s			=	G_StaV_St.Last_ACQ_s;			//�ϴ�ͬ���������s
	UART_SysSTAVar->Last_ACQ_ns			=	G_StaV_St.Last_ACQ_ns;			//�ϴ�ͬ���������ns
	UART_SysSTAVar->Last_ADJ_s			=	G_StaV_St.Last_ADJ_s;			//�ϴ�ͬ������s
	UART_SysSTAVar->Last_ADJ_ns			=	G_StaV_St.Last_ADJ_ns;			//�ϴ�ͬ������s
	UART_SysSTAVar->Auto_SYN			=	G_StaV_St.Auto_SYN;				//��ͬ��ʹ��
	UART_SysSTAVar->Auto_OCXOADJ		=	G_StaV_St.Auto_OCXOADJ;			//�Ե�������ʹ��
	UART_SysSTAVar->Prog_Pul			=	G_StaV_St.Prog_Pul;				//�ɱ������ʹ��
	UART_SysSTAVar->Auto_KeepSIn		=	G_StaV_St.Auto_KeepSIn;			//���л���ʱ״̬ʹ��
	UART_SysSTAVar->Meas_Pul			=	G_StaV_St.Meas_Pul;				//��������ʹ��
	UART_SysSTAVar->Prog_PerT			=	G_StaV_St.Prog_PerT;			//�ɱ����������
	UART_SysSTAVar->Messa_SYN			=	G_StaV_St.Messa_SYN;			//ͬ����Ϣ���ʹ��
	UART_SysSTAVar->Messa_1PPS			=	G_StaV_St.Messa_1PPS;			//������������Ϣʹ��
	UART_SysSTAVar->Messa_PGP			=	G_StaV_St.Messa_PGP;			//�ɱ��������Ϣʹ��
	UART_SysSTAVar->Messa_GPS			=	G_StaV_St.Messa_GPS;			//GPS ����ת��ʹ��

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ sizeof(UART_SysSTAVarSt) + 5 ] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ sizeof(UART_SysSTAVarSt) + 6 ] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ���ʹ�������֡
///
//////////////////////////////////////////////////////
void Send_Err(u8 ErrCMD, u8 * ErrData, u8 Len)
{
	u16 i;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'e' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] = Len + 1  ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5] = ErrCMD ;

	for(i = 0; i < Len; i++)
		U6_Sys_TxFrame[U6_Sys_Tx_InFrame][6 + i] = ErrData[i];

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ Len + 1 + 5 ] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ Len + 1 + 6 ] = 0x0A ;
	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ���ͱ�����������Ϣ����
///
//////////////////////////////////////////////////////
void Send_NaviP(void)
{
	u8	InfoSize ;
	UART_NaviVarSt UART_NaviVar ;

	InfoSize	=	sizeof(UART_NaviVarSt) ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'p' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] = sizeof(UART_NaviVarSt)  ;

	UART_NaviVar.TimeKeepSTA 	=	G_StaV_St.TimeKeepSTA ;         		//��ѭ��״̬��
	UART_NaviVar.TKS_Flag		=	G_StaV_St.TKS_Flag;        				//��ʱ�ȶ���ʶ
	UART_NaviVar.NATI_Time		= 	G_StaV_St.NATI_Time;					//����1PPSʱ��
	UART_NaviVar.GPS_STA		=	G_StaV_St.GPS_STA;         				//GPS״̬��ʾ
	UART_NaviVar.GPS_Time		= 	G_StaV_St.GPS_Time;               		//GPSʱ��
	UART_NaviVar.GPS_longitude	=	(s32)(G_StaV_St.GPS_longitude * 10000000); //GPS����
	UART_NaviVar.GPS_latitude	= 	(s32)(G_StaV_St.GPS_latitude * 10000000); //GPSγ��

	*((UART_NaviVarSt *)(&(U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5]))) = UART_NaviVar ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ InfoSize + 5 ] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ InfoSize + 6 ] = 0x0A ;
	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ���ͽ�5�β���ͬ�����
///
//////////////////////////////////////////////////////
void Send_Last_SYN_Table(SYN_Info * SYN_Info_p, u8 Next_SYN_Item)
{
	u16 i;
	u8	InfoSize ;
	InfoSize	=	sizeof(SYN_Info) ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'l' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] = sizeof(SYN_Info)  ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] *= 5;

	for(i = 0; i < 5 ; i++)
	{
		*((SYN_Info *)(&(U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5 + i * InfoSize]))) = SYN_Info_p[(Next_SYN_Item + 4 - i) % 5] ;
	}

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ InfoSize * 5 + 5 ] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ InfoSize * 5 + 6 ] = 0x0A ;
	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ����ͬ����ʱ���
///
//////////////////////////////////////////////////////
void Send_Meas_Adj_Res(void)
{
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'a' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] =  20 ;

	*((u32 *) & (U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5]))  = G_StaV_St.Last_SYN_Time ; //�ϴ�ͬ��ʱ��
	*((s32 *) & (U6_Sys_TxFrame[U6_Sys_Tx_InFrame][9]))  = G_StaV_St.Last_ACQ_s ;
	*((s32 *) & (U6_Sys_TxFrame[U6_Sys_Tx_InFrame][13])) = G_StaV_St.Last_ACQ_ns ;
	*((s32 *) & (U6_Sys_TxFrame[U6_Sys_Tx_InFrame][17])) = G_StaV_St.Last_ADJ_s ;
	*((s32 *) & (U6_Sys_TxFrame[U6_Sys_Tx_InFrame][21])) = G_StaV_St.Last_ADJ_ns ;


	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][25] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][26] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ��������汾��
///
//////////////////////////////////////////////////////
void Send_SoftVer(void)
{
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'u' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] = 2   ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5] = (u8)Soft_Ver ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][6] = (u8)Hard_Ver ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][7] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][8] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ��оƬ����
///
//////////////////////////////////////////////////////
void Send_SIGID(void)
{
	u32 Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(vu32*)(0x1FFF7A10);      //12 Bytes Serial Number
	Device_Serial1 = *(vu32*)(0x1FFF7A14);
	Device_Serial2 = *(vu32*)(0x1FFF7A28);

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'z' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] = 12  ;

	*((u32*)(&U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5]))  = Device_Serial0 ;
	*((u32*)(&U6_Sys_TxFrame[U6_Sys_Tx_InFrame][9]))  = Device_Serial1 ;
	*((u32*)(&U6_Sys_TxFrame[U6_Sys_Tx_InFrame][13])) = Device_Serial2 ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][17] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][18] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ת��GPS���ֱ��ĵ�PC
///
//////////////////////////////////////////////////////
void Send_GPS_C_V(u8* GPSdata)
{
	u8 i;

	if(GPSdata[0] > 200 )
		return;

	if(GPSdata[5] != 'C' && GPSdata[5] != 'V')
		return;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'g' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] = GPSdata[0]  ;

	for(i = 0; i < GPSdata[0]; i++)
		U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5 + i] = GPSdata[i] ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5] = '$';

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][GPSdata[0] + 5] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][GPSdata[0] + 6] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ����Э��Ҫ�������֡������s
///
//////////////////////////////////////////////////////
void Send_Sys_NavPPS(void)
{
	SyFrame	* SyFrameP ;
	RTC_TIME_Str 	RTC_TIME_Var ;

	RTC_TIME_Var.Unix_Time = G_StaV_St.NATI_Time ;

	UnixT2Rtc(&RTC_TIME_Var);

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'R' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] =  32 ;

	SyFrameP	=	(SyFrame *)&U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5];

	SyFrameP->fHead	=	0x7E ;
	SyFrameP->fType = 	0x00 ;

	SyFrameP->messData.uiYear		=	RTC_TIME_Var.QYear;	//��(4λʮ������ֵ):>1900
	SyFrameP->messData.ucMonth		=	RTC_TIME_Var.Mon;	//��:1-12
	SyFrameP->messData.ucDay		=	RTC_TIME_Var.Dat ;	//��:1-31
	SyFrameP->messData.ucWeek		=	RTC_TIME_Var.Day ;	//week 1-7     add week
	SyFrameP->messData.ucHour		=	RTC_TIME_Var.Hor;	//24Сʱ��:0-23
	SyFrameP->messData.ucMinute		=	RTC_TIME_Var.Min;	//��:0-59
	SyFrameP->messData.ucSecond		=	RTC_TIME_Var.Sec;	//��:0-59
	//ms������<=1000
	SyFrameP->messData.usCounterms	=	0;

	/*
	����
	��ʽ:N39.7172520000,E116.9242730000
	*/
	if(G_StaV_St.GPS_latitude > 0)
		SyFrameP->messData.NorS			=	'N';			//'N'/'n'��γ;'S'/'s'��γ
	else
		SyFrameP->messData.NorS			=	'S';

	if(G_StaV_St.GPS_longitude > 0)
		SyFrameP->messData.EorW			=	'E';			//'E'/'e'����;'W'/'w'����
	else
		SyFrameP->messData.EorW			=	'W';

	SyFrameP->messData.dLatitude	=	G_StaV_St.GPS_latitude;		//γ��  double float
	SyFrameP->messData.dLongitude	=	G_StaV_St.GPS_longitude;  	//����

	SyFrameP->crc16Check			=	getCRC16((u8 *)SyFrameP, sizeof(SyFrame) - 2, crcTalbe) ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 5] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 6] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ����Э��Ҫ�������֡��GPSPPS
///
//////////////////////////////////////////////////////
void Send_Sys_GPSPPS(void)
{
	SyFrame	* SyFrameP ;
	RTC_TIME_Str 	RTC_TIME_Var ;

	RTC_TIME_Var.Unix_Time = G_StaV_St.GPS_Time ;

	UnixT2Rtc(&RTC_TIME_Var);

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'R' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] =  32 ;

	SyFrameP	=	(SyFrame *)&U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5];

	SyFrameP->fHead	=	0x7E ;
	SyFrameP->fType = 	0x03 ;

	SyFrameP->messData.uiYear		=	RTC_TIME_Var.QYear;	//��(4λʮ������ֵ):>1900
	SyFrameP->messData.ucMonth		=	RTC_TIME_Var.Mon;	//��:1-12
	SyFrameP->messData.ucDay		=	RTC_TIME_Var.Dat ;	//��:1-31
	SyFrameP->messData.ucWeek		=	RTC_TIME_Var.Day ;	//week 1-7     add week
	SyFrameP->messData.ucHour		=	RTC_TIME_Var.Hor;	//24Сʱ��:0-23
	SyFrameP->messData.ucMinute		=	RTC_TIME_Var.Min;	//��:0-59
	SyFrameP->messData.ucSecond		=	RTC_TIME_Var.Sec;	//��:0-59
	//ms������<=1000
	SyFrameP->messData.usCounterms	=	0;

	/*
	����
	��ʽ:N39.7172520000,E116.9242730000
	*/
	if(G_StaV_St.GPS_latitude > 0)
		SyFrameP->messData.NorS			=	'N';			//'N'/'n'��γ;'S'/'s'��γ
	else
		SyFrameP->messData.NorS			=	'S';

	if(G_StaV_St.GPS_longitude > 0)
		SyFrameP->messData.EorW			=	'E';			//'E'/'e'����;'W'/'w'����
	else
		SyFrameP->messData.EorW			=	'W';

	SyFrameP->messData.dLatitude	=	G_StaV_St.GPS_latitude;		//γ��  double float
	SyFrameP->messData.dLongitude	=	G_StaV_St.GPS_longitude;  	//����

	SyFrameP->crc16Check			=	getCRC16((u8 *)SyFrameP, sizeof(SyFrame) - 2, crcTalbe) ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 5] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 6] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  ����Э��Ҫ�������֡����������ʱ������
///
//////////////////////////////////////////////////////
void Send_Sys_MeasPlu(u32 Mea_s, u32 Mea_ns)
{
	SyFrame	* SyFrameP ;
	RTC_TIME_Str 	RTC_TIME_Var ;

	Mea_ns					=	Mea_ns * 20 ;				//20ns��λ�����ns	����ʱ��ns
	RTC_TIME_Var.Unix_Time 	= 	Mea_s ;					//����ʱ����s

	UnixT2Rtc(&RTC_TIME_Var);							//unixʱ��תΪRTCʱ��

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;		//��ʼ
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;		//Ŀ�ĵ�ַ ��P����λ��
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;		//Դ��ַ	  ��T����ʱģ��
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'R' ;		//������		R������ͬҪ����
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] =  32 ; 		//�����򳤶ȣ�32���ֽ�

	SyFrameP	=	(SyFrame *)&U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5];

	SyFrameP->fHead	=	0x7E ;																	//���֡ͷ
	SyFrameP->fType = 	0x01 ;																	//֡���ͣ���������ʱ��

	SyFrameP->messData.uiYear		=	RTC_TIME_Var.QYear;	//��(4λʮ������ֵ):>1900
	SyFrameP->messData.ucMonth		=	RTC_TIME_Var.Mon;	//��:1-12
	SyFrameP->messData.ucDay		=	RTC_TIME_Var.Dat ;	//��:1-31
	SyFrameP->messData.ucWeek		=	RTC_TIME_Var.Day ;	//week 1-7     add week
	SyFrameP->messData.ucHour		=	RTC_TIME_Var.Hor;	//24Сʱ��:0-23
	SyFrameP->messData.ucMinute		=	RTC_TIME_Var.Min;	//��:0-59
	SyFrameP->messData.ucSecond		=	RTC_TIME_Var.Sec;	//��:0-59

	//ms������<=1000
	SyFrameP->messData.usCounterms	=	Mea_ns / 1000000;

	/*
	����
	��ʽ:N39.7172520000,E116.9242730000															//�����㲻ȡ��Ҳ�������GPS��Ч�������������һ����Чλ����Ϣ
	*/
	if(G_StaV_St.GPS_latitude > 0)
		SyFrameP->messData.NorS			=	'N';			//'N'/'n'��γ;'S'/'s'��γ
	else
		SyFrameP->messData.NorS			=	'S';

	if(G_StaV_St.GPS_longitude > 0)
		SyFrameP->messData.EorW			=	'E';			//'E'/'e'����;'W'/'w'����
	else
		SyFrameP->messData.EorW			=	'W';

	SyFrameP->messData.dLatitude	=	G_StaV_St.GPS_latitude;		//γ��
	SyFrameP->messData.dLongitude	=	G_StaV_St.GPS_longitude;  	//����

	SyFrameP->crc16Check			=	getCRC16((u8 *)SyFrameP, sizeof(SyFrame) - 2, crcTalbe) ;	//���CRC16У��

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 5] = 0xFF ;								//�ҵ�BCCУ�飬��ʱΪ0xFF���ڷ��䴦��ʱ���
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 6] = 0x0A ;								//�ҵĽ����ֽ�

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}

///////////////////////////////////////////////////////////
/////����PC���Ĵ��ڱ�������
extern u8  Err_Temp ;				//��ʱ�����ʶ
extern u8  ProgP_SYN_Flag;			//ͬ��������Ҫ����ͬ���ı�־λ
extern SYN_Info	Last_SYN_Table[5];	//���5��ͬ����Ϣ��¼��
extern u8		Next_SYN_Item;		//ָ��һ�����ϵ�ͬ���������Ϣ

char ProcPC_CMD(u8 New_CMD_Flag, u8 * New_Data )
{
	u32		Prog_PerT_temp ;			//�ɱ��������������ʱ����Ҫ�õ�����ʱ����

	if( New_CMD_Flag == 'T')			//��ѯ״̬
	{
		New_CMD_Flag = 0 ;
		//��λ����ѯ״̬
		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'O')							//���ú��¾���
	{
		New_CMD_Flag = 0 ;

		if( *((u16*)(&New_Data[0])) <= 1023 && *((u16*)(&New_Data[0])) >= 10 )
		{
			G_StaV_St.DigPot = *((u16*)(&New_Data[0])) ;
			DIGPOT_Write(G_StaV_St.DigPot);
			Send_STA_Sys();                     			//���ǵ����ָ���õĲ���,����ֱ�ӻظ�״̬,
		}
		else
		{
			Err_Temp = 2;
			Send_Err('O', &Err_Temp, 2);          			//�������ʱ������,����1
		}
	}
	else if(New_CMD_Flag == 'B')							//ѹ���Ե���,���ݣ�ÿ2���ӵ�Ư�����������Ƿ���ĺ��¾����ѹ
	{
		//Ҫ���㣬�Ѿ���������40min��GPS�����ȶ� ��Щ����
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Auto_OCXOADJ != 0)						//��תѹ���Ե���
			G_StaV_St.Auto_OCXOADJ	= 0;
		else
			G_StaV_St.Auto_OCXOADJ	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'A')							//��ͬ������GPS����������������Ч�����Զ����ʱ��ͬ��
	{
		//���ڳ�ʼ״̬��ͬ��״̬���Զ�ͬ��������
		New_CMD_Flag = 0 ;									//����ʱ״̬�£�Ĭ��GPSģ��ϵ�

		//����ʱ״̬�£���GPS���ֶ��򿪣�������Զ�����������ͬ����ֻ����Ư����
		if(G_StaV_St.Auto_SYN != 0)		//��ת��ͬ��			//���Զ�ͬ�����ܹرգ���GPS�����ź��ȶ���ÿ2min������Ư�����1��
			G_StaV_St.Auto_SYN	= 0;
		else
			G_StaV_St.Auto_SYN	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'G')		//GPS��Դ			//�л�״̬ʱ�������״̬��ȷ��GPS��Դ״̬��Ȼ�󣬿��ֶ��رջ��,�򿪺�������λ����������
	{
		New_CMD_Flag = 0 ;
		/*
				�˴����ܣ�Ǩ����IO�ڣ����ٴ����д���״̬�������߼�����
				if(GPS_Power != 0)				//��תGPS��Դ
				{
					GPS_Power			= 	0;
					G_StaV_St.GPS_STA	=	GPS_RNSS_OFF ;
				}
				else
				{
					GPS_Power			= 	1;
					G_StaV_St.GPS_STA	=	GPS_RNSS_V ;
					G_StaV_St.TKS_Flag 	= 	0 ;						//��GPS����������
					TKPSTALED			= 	LED_Flash_100Ms;
				}

				GPS_Pow(GPS_Power);
				Send_STA_Sys();
		*/
		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'C')		//ͬ����Ϣ���
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_SYN != 0)	//��תͬ����Ϣ���
			G_StaV_St.Messa_SYN	= 0;
		else
			G_StaV_St.Messa_SYN	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'P')		//������������Ϣ���
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_1PPS != 0)	//��ת������������Ϣ���
			G_StaV_St.Messa_1PPS = 0;
		else
			G_StaV_St.Messa_1PPS = 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'W')		//�ɱ��������Ϣ���
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_PGP != 0)	//��ת�ɱ��������Ϣ���
			G_StaV_St.Messa_PGP	= 0;
		else
			G_StaV_St.Messa_PGP	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'V')		//�ɱ���������
	{
		New_CMD_Flag = 0 ;

		Prog_PerT_temp = *((u32*)(&New_Data[0])) ;					//�Ѵ�������������ȡ����

		if( G_StaV_St.Prog_PerT + 1 != MS2ON20NS(Prog_PerT_temp)	//�͵�ǰ�Ĳ�һ��
				&&	Prog_PerT_temp % 100 	== 0							//��0.1s��������
				&&	Prog_PerT_temp		 	<= 60000						//С�ڵ���60s
				&&	Prog_PerT_temp		 	>=  1000)						//���ڵ���1s
		{
			G_StaV_St.Prog_PerT = MS2ON20NS(Prog_PerT_temp) - 1;

			ProgP_SYN_Flag 	= 1 ;									//���������ڲ���������׼������ͬ���ɱ����������
		}

		if(New_Data[4]  == 'A')
		{
			if(G_StaV_St.Prog_Pul == 0)
			{
				G_StaV_St.Prog_Pul	= 1;
				ProgP_SYN_Flag 		= 1 ;
			}
		}
		else
		{
			G_StaV_St.Prog_Pul	= 0;
			ProgP_SYN_Flag 		= 0 ;
		}

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'N')		//���л���ʱ״̬ʹ��
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Auto_KeepSIn != 0)	//��ת���л���ʱ״̬ʹ��
			G_StaV_St.Auto_KeepSIn = 0;
		else
			G_StaV_St.Auto_KeepSIn = 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'M')		//�����������ʹ��
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Meas_Pul	!= 0)	//��ת�����������ʹ��
			G_StaV_St.Meas_Pul	 = 0;
		else
			G_StaV_St.Meas_Pul	 = 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'Y')		//GPS����ת��
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_GPS != 0)	//�򿪻�ر�GPS����ת��
			G_StaV_St.Messa_GPS	= 0;
		else
			G_StaV_St.Messa_GPS	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'L')		//���5��ͬ����������
	{

		New_CMD_Flag = 0 ;
		Send_Last_SYN_Table(&(Last_SYN_Table[0]), Next_SYN_Item);

	}
	else if(New_CMD_Flag == 'Z')		//�ָ���������
	{
		New_CMD_Flag = 0 ;

		G_StaV_St.DigPot		= 513;             		//���ֵ�λ��ֵ��Ĭ��Ϊ513
		G_StaV_St.Auto_SYN		= 1;					//��ͬ��ʹ��
		G_StaV_St.Auto_OCXOADJ	= 0;					//�Ե�������ʹ��
		G_StaV_St.Prog_Pul		= 0;					//�ɱ������ʹ��
		G_StaV_St.Auto_KeepSIn	= 0;					//���л���ʱ״̬ʹ��
		G_StaV_St.Meas_Pul		= 1;					//��������ʹ��
		G_StaV_St.Prog_PerT		= MS2ON20NS(1600) - 1;	//�ɱ����������
		G_StaV_St.Messa_SYN		= 1;					//ͬ����Ϣ���ʹ��
		G_StaV_St.Messa_1PPS	= 1;					//������������Ϣʹ��
		G_StaV_St.Messa_PGP		= 0;					//�ɱ��������Ϣʹ��
		G_StaV_St.Messa_GPS		= 0;					//GPS ����ת��ʹ��

		Set_Para_IIC(&G_StaV_St);

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'K')		//������ʱ��������
	{
		New_CMD_Flag = 0 ;
		//Set_Para(&G_StaV_St);
		Set_Para_IIC(&G_StaV_St);
		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'U')		//��ȡ�汾��
	{
		New_CMD_Flag = 0 ;
		Send_SoftVer();
	}
	else if(New_CMD_Flag == 'H')		//��ȡ�汾��
	{
		New_CMD_Flag = 0 ;
		Send_SIGID();
	}

	//ȱ�١�S���л�״̬  ��Q��ǿ��ͬ��

	return New_CMD_Flag;
}
