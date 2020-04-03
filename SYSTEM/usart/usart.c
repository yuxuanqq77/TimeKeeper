#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "DIGPOT.h"
#include "GPS.h"
#include "stmflash.h"
#include "24cxx.h"

extern SYS_GSV		G_StaV_St ;

//串口接收数据缓冲区，DMA自动填充，循环缓冲
#include <stm32f4xx.h>
__align(4) u8  U6_Sys_RxBuf[UARTRXBUFLEN] ;
u16 U6_Sys_Rx_ProcPos = 0;          //指向还没有处理的字节
u16 U6_Sys_Rx_GetPos  = 0;          //指向还没有被填充的字节，在接收处理函数中更新

//解析出来的有效帧
__align(4) u8  U6_Sys_RxFrame[8][512] ;
u16 U6_Sys_Rx_ProcFrame = 0;        //指向还没有处理的有效帧
u16 U6_Sys_Rx_GetFrame  = 0;        //指向还没有被解析出来的空帧

//发射的帧
__align(4) u8  U6_Sys_TxFrame[20][512] ;
u16 U6_Sys_Tx_InFrame = 0;          //指向还没有被（主程序）填充的帧，空帧
u16 U6_Sys_Tx_OuFrame = 0;          //指向还没有发射出的帧

//////////////////////////////////////
////  UART6 RX 使用的DMA
////  DMA2 Stream1 Ch5 初始化
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

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; //DMA2时钟使能

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 	//关闭DMA传输

	while(1)
	{
		if( (DMA_Streamx->CR & DMA_SxCR_EN) == 0 )
			break;
	}

	streamx = (((u32)DMA_Streamx - (u32)DMAx) - 0X10) / 0X18;		//得到stream通道号

	if(streamx >= 6)
		DMAx->HIFCR |= 0X3D << (6 * (streamx - 6) + 16);					//清空之前该stream上的所有中断标志
	else if(streamx >= 4)
		DMAx->HIFCR |= 0X3D << 6 * (streamx - 4);    					//清空之前该stream上的所有中断标志
	else if(streamx >= 2)
		DMAx->LIFCR |= 0X3D << (6 * (streamx - 2) + 16);					//清空之前该stream上的所有中断标志
	else
		DMAx->LIFCR |= 0X3D << 6 * streamx;							//清空之前该stream上的所有中断标志

	DMA_Streamx->PAR  = (u32) & (USART6->DR);		 		//DMA外设地址
	DMA_Streamx->M0AR = (u32)U6_Sys_RxBuf ;		 		//DMA 存储器0地址
	DMA_Streamx->NDTR = UARTRXBUFLEN;		       		//
	DMA_Streamx->CR   = 0;			                	//先全部复位CR寄存器值

	DMA_Streamx->CR  &= ~DMA_SxCR_DIR;		   //外设到存储器模式
	DMA_Streamx->CR  |= DMA_SxCR_CIRC;	    	//循环模式
	DMA_Streamx->CR  &= ~DMA_SxCR_PINC;		  	//外设非增量模式
	DMA_Streamx->CR  |= DMA_SxCR_MINC;		  	//存储器增量模式
	DMA_Streamx->CR  &= ~DMA_SxCR_PSIZE;		  	//外设数据长度:8位
	DMA_Streamx->CR  &= ~DMA_SxCR_MSIZE;		  	//存储器数据长度:8位
	DMA_Streamx->CR  &= ~DMA_SxCR_PL;		   //低优先级
	DMA_Streamx->CR  &= ~DMA_SxCR_PBURST;		//外设突发单次传输
	DMA_Streamx->CR  &= ~DMA_SxCR_MBURST;		//存储器突发单次传输

	DMA_Streamx->CR  |= (u32)chx << 25;        	//通道选择

	DMA_Streamx->CR  &= ~DMA_SxCR_TCIE ;     	//不要中断

	DMA_Streamx->CR  |= DMA_SxCR_EN;        	//打开DMA
}
//////////////////////////////////////
////  UART6 TX 使用的DMA
////  DMA2 Stream6 Ch5 初始化
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

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;               //DMA2时钟使能

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 	              //关闭DMA传输

	while(1)
	{
		if( (DMA_Streamx->CR & DMA_SxCR_EN) == 0 )
			break;
	}

	streamx = (((u32)DMA_Streamx - (u32)DMAx) - 0X10) / 0X18;	//得到stream通道号

	if(streamx >= 6)
		DMAx->HIFCR |= 0X3D << (6 * (streamx - 6) + 16);	      		//清空之前该stream上的所有中断标志
	else if(streamx >= 4)
		DMAx->HIFCR |= 0X3D << 6 * (streamx - 4);                 	//清空之前该stream上的所有中断标志
	else if(streamx >= 2)
		DMAx->LIFCR |= 0X3D << (6 * (streamx - 2) + 16);            	//清空之前该stream上的所有中断标志
	else
		DMAx->LIFCR |= 0X3D << 6 * streamx;						//清空之前该stream上的所有中断标志

	DMA_Streamx->PAR  = (u32) & (USART6->DR);		  		//DMA外设地址
	DMA_Streamx->M0AR = (u32)0 ;		           		//DMA 存储器0地址
	DMA_Streamx->NDTR = 100;		              		//
	DMA_Streamx->CR   = 0;			            		//先全部复位CR寄存器值

	DMA_Streamx->CR  |= DMA_SxCR_DIR_0;		  			//存储器到外设模式
	DMA_Streamx->CR  &= ~DMA_SxCR_CIRC;					//非循环模式
	DMA_Streamx->CR  &= ~DMA_SxCR_PINC;					//外设非增量模式
	DMA_Streamx->CR  |= DMA_SxCR_MINC;					//存储器增量模式
	DMA_Streamx->CR  &= ~DMA_SxCR_PSIZE;		  			//外设数据长度:8位
	DMA_Streamx->CR  &= ~DMA_SxCR_MSIZE;		  			//存储器数据长度:8位
	DMA_Streamx->CR  &= ~DMA_SxCR_PL;		    		//低优先级
	DMA_Streamx->CR  &= ~DMA_SxCR_PBURST;				//外设突发单次传输
	DMA_Streamx->CR  &= ~DMA_SxCR_MBURST;				//存储器突发单次传输

	DMA_Streamx->CR  |= (u32)chx << 25;          			//通道选择
	DMA_Streamx->CR  &= ~DMA_SxCR_TCIE ;       			//关中断
	DMA_Streamx->CR  &= ~DMA_SxCR_EN;          			//不启动，等待发送的时候再启动
}

//设置发送1帧数据启动对应的DMA数据流
void Enable_U6_Tx_DMA(u8 * Tx_Buf, u16 Tx_Num)
{
	DMA_Stream_TypeDef * DMA_Streamx ;
	DMA_TypeDef *DMAx;
	u8 streamx;

	DMA_Streamx = DMA2_Stream6 ;
	DMAx        = DMA2 ;
	streamx     = 6 ;

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 	//关闭DMA传输

	while(1)
	{
		if( (DMA_Streamx->CR & DMA_SxCR_EN) == 0 )
			break;
	}

	streamx = (((u32)DMA_Streamx - (u32)DMAx) - 0X10) / 0X18;	//得到stream通道号

	if(streamx >= 6)
		DMAx->HIFCR |= 0X3D << (6 * (streamx - 6) + 16);	          	//清空之前该stream上的所有中断标志
	else if(streamx >= 4)
		DMAx->HIFCR |= 0X3D << 6 * (streamx - 4);                 	//清空之前该stream上的所有中断标志
	else if(streamx >= 2)
		DMAx->LIFCR |= 0X3D << (6 * (streamx - 2) + 16);            	//清空之前该stream上的所有中断标志
	else
		DMAx->LIFCR |= 0X3D << 6 * streamx;						//清空之前该stream上的所有中断标志

	DMA_Streamx->M0AR = (u32)Tx_Buf ;
	DMA_Streamx->NDTR = Tx_Num ;

	DMA_Streamx->CR |= DMA_SxCR_EN;                     //启动数据传输，传输完毕，自动停止，非循环模式
}

//////////////////////////////////////////////////////////////////////////////////
//// UART6向SYS单片机的串口初始化
//// 这是一个很高科技的串口驱动
////
//// 输入使用DMA送入环形缓冲区，DMA循环运行,
//// 靠DMA的传输个数寄存器和已经处理的字节位置,
//// 判断是否有需要处理的字节。
//// 如果有字节没有处理，就读出来送入串口状态机循环
//// 解析出来有效帧。送入有效帧缓冲区。等待主循环处理。
////
//// 输出也使用DMA。通过最新加入发送域和最老送入发送域两个变量，
//// 判断是否需要开始传送。如果有，则将最老帧用DMA发送出去。
////
//////////////////////////////////////////////////////////////////////////////////
void UART6_SYS_init(u32 pclk2, u32 bound)
{
	float temp;

	u16 mantissa;
	u16 fraction;

	UART6_RX_DMA_Config();                  //配置循环接收DMA，并启动

	UART6_TX_DMA_Config();                  //配置发送DMA，但不启动，只是配置DMA模式

	temp = (float)(pclk2 * 1000000) / (bound * 16); //得到USARTDIV@OVER8=0
	mantissa = temp;				                //得到整数部分
	fraction = (temp - mantissa) * 16;      //得到小数部分@OVER8=0
	mantissa <<= 4;
	mantissa += fraction;

	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN ;   	//使能PORTC口时钟
	RCC->APB2ENR  |= RCC_APB2ENR_USART6EN;  	//使能串口6时钟

	RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;    //复位串口6
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;

	GPIO_Set(GPIOC, PIN6 | PIN7, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE); //PC6,PC7,复用功能,上拉输出
	GPIO_AF_Set(GPIOC, 6, 8);	//PC6,AF8
	GPIO_AF_Set(GPIOC, 7, 8); //PC7,AF8

	//波特率设置
	USART6->BRR = mantissa; 	            //波特率设置
	USART6->CR1 &= ~(1 << 15); 	         //设置OVER8=0

	USART6->CR3 |= USART_CR3_DMAT;          //接收DMA使能
	USART6->CR1 |= USART_CR1_TE;  	        //串口发送使能

	mantissa = USART6->DR ;                 //清空一下DR寄存器

	USART6->CR3 |= USART_CR3_DMAR;          //发送DMA使能
	USART6->CR1 |= USART_CR1_RE;  	        //串口接收使能

	USART6->CR1 |= USART_CR1_UE;  	        //串口使能
}

////////////////////
/// 串口状态机
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
/// 串口通信地址
////////////////////
#define SIGUADD 'S'   //信号单片机
#define TIMUADD 'T'   //守时单片机
#define SYSUADD 'M'   //主控单片机
#define PCZUADD 'P'   //上位机

u32 UART6_SYS_OveTimeS ;        //超时计数器变量
u8  UART6_SYS_OveTime50MS; 		//

//异或和校验计算，协议只定义从'$'到BCC字节之间的所有待传送字节异或
u8 CheckBcc(u8 * CheckBuf, u16 CheckLen)
{
	u16 i;
	u8  TempBcc = 0 ;

	for(i = 0; i < CheckLen; i++)
		TempBcc = TempBcc ^ CheckBuf[i];

	return TempBcc ;
}

//////////////////////////////////////////
//// 接收循环缓冲区，数据解析为有效帧循环
//// 这是接收程序最为复杂的函数。
//// 配有超时处理
u16 Proc_RxByte_SYS(void)
{
	static u8  Uart6_Sys_STA  = UART_IDLE ;
	static u16 RxProPos       = 0 ;
	static u16 DataLen        = 0 ;

	static u8  UART6_SYS_OveTime_Flag = 1 ;//超时标志位 1超时或空闲，0启动超时判断

	u16 i = 0 ;
	u8  ReaD ;

	//串口超时，从收到‘$’开始计时，超过1s认为超时，恢复到空闲状态
	if( UART6_SYS_OveTime_Flag != 1 )
	{
		if( CheckTimerOve(UART6_SYS_OveTimeS, UART6_SYS_OveTime50MS) == 1
				&&  Uart6_Sys_STA != UART_IDLE )
		{
			Uart6_Sys_STA = UART_IDLE ;
			UART6_SYS_OveTime_Flag = 1;
		}
	}

	//取出现在接收到哪个字节位置
	//U6_Sys_Rx_GetPos始终指向一个DMA还没有用的位置，空数据
	//U6_Sys_Rx_ProcPos始终指向一个还没有处理的位置
	//两个变量一样，说明，没有需要处理的字节，这里没有考虑突然来了很多字节
	//循环溢出的错误。
	//只是认为接收缓冲区足够大，大到不可能出现循环溢出错误

	U6_Sys_Rx_GetPos = ( UARTRXBUFLEN - DMA2_Stream1->NDTR ) ;

	//最多处理10个字节，避免时间太长，影响主循环
	while(i < 10)
	{
		if(U6_Sys_Rx_ProcPos == U6_Sys_Rx_GetPos)     	//没有要处理数据
			break;
		else
		{
			ReaD = U6_Sys_RxBuf[U6_Sys_Rx_ProcPos] ;    //有要处理的字节，取出

			U6_Sys_Rx_ProcPos++;                        //取出后循环+1
			U6_Sys_Rx_ProcPos %= UARTRXBUFLEN;

			switch(Uart6_Sys_STA)
			{
				case UART_IDLE:                           //空闲状态，只有收到$，才会跳出

					if(ReaD == '$')
					{
						RxProPos = 0 ;
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						UART6_SYS_OveTime_Flag = 0 ;                //开始超时计数

						UART6_SYS_OveTimeS   	= 	G_StaV_St.RunTime + 1 ;        	//设置超时时间1s
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
					if(ReaD == TIMUADD)                    		//目的地址判断，是否是信号单片机
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

				case UART_ST2:                            		//源地址判断，是否是主控单片机
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

				case UART_ST3:                                //命令域是否是有效命令
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

				case UART_ST4:                                		//数据域长度是否正确
					if(ReaD <= 250)                             	//0-200，数据域长度为0-200
					{
						//201为 200+(201-200)*2 = 202
						if(ReaD <= 200)                           	//250为 200+(250-200)*2 = 300
							DataLen = ReaD ;                        //大于250，判断为错误
						else
							DataLen = 2 * (ReaD - 200) + 200 ;

						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						if( DataLen == 0 )                        	//如果，数据域长度为0，直接跳到BCC校验
							Uart6_Sys_STA = UART_ST6 ;          	//否则跳到数据域接收
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
					if( DataLen != 0 )                          //这里其实不用判断，为了好看而已
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;
						DataLen--;
					}

					if( DataLen == 0 )                      	//如果数据域接收完毕，那么跳到BCC检查
						Uart6_Sys_STA = UART_ST6 ;
					else
						Uart6_Sys_STA = UART_ST5 ;

					break;

				case UART_ST6:                                	//BCC检查
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

				case UART_ST7:                                	//验证是否有0x0A,有则成功收到报文

					if(ReaD == 0x0A)
					{
						U6_Sys_RxFrame[U6_Sys_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						//这里更新有效帧
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

	//返回还有多少个字节没有处理
	return (U6_Sys_Rx_GetPos + UARTRXBUFLEN - U6_Sys_Rx_ProcPos) % UARTRXBUFLEN ;
}
//

//////////////////////////////////////////////////////
///      发送数据帧
///
//////////////////////////////////////////////////////
void Proc_TxFrame_SYS(void)
{
	u8 FrameTxNum ;

	if(   U6_Sys_Tx_InFrame != U6_Sys_Tx_OuFrame    	//确认有数据帧需要发送
			&& (DMA2_Stream6->CR  &  DMA_SxCR_EN ) == 0 ) 		//确认目前没有在发送数据，DMA数据流已经自动停止了。
	{
		if( U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][0] == '$' //确认除BCC字节外，所有域数据正确
				&& U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][1] == 'P'
				&& U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][2] == 'T'
				&& ((U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] <= 'Z' && U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] >= 'A') ||
					(U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] <= 'z' && U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][3] >= 'a'))
				&& U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] <= 250)
		{
			//计算帧长度
			if(U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] >= 200)
				FrameTxNum = (U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] - 200) * 2 + 250 + 7;
			else
				FrameTxNum = U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][4] + 7;

			if( U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][FrameTxNum - 1] == 0x0A )
			{
				U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][FrameTxNum - 2]
					= CheckBcc(&U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][0], FrameTxNum - 2);       		//加入BCC

				Enable_U6_Tx_DMA( &U6_Sys_TxFrame[U6_Sys_Tx_OuFrame][0], FrameTxNum );    	//启动DMA传输
			}
		}

		//如果数据帧开始发送，更新已发送指向
		//如果数据帧没有发送，也要更新，说明待发送的数据帧有错误，跳过不发送
		U6_Sys_Tx_OuFrame++;
		U6_Sys_Tx_OuFrame %= 20;
	}
}
//////////////////////////////////////////////////////
///  主程序中取出一个最老的有效帧，命令及数据
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

		//DataLen最大不会超过300字节
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
///  发送TK状态
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

	UART_SysSTAVar->RunTime				=	G_StaV_St.RunTime;             	//运行时钟
	UART_SysSTAVar->RunTime_50ms		=	G_StaV_St.RunTime_50ms;   		//50ms为单位的运行时钟
	UART_SysSTAVar->TimeKeepSTA			=	G_StaV_St.TimeKeepSTA;         	//主循环状态机
	UART_SysSTAVar->Temper				=	G_StaV_St.Temper;          		//当前芯片温度
	UART_SysSTAVar->DigPot				=	G_StaV_St.DigPot;             	//数字电位器值，默认为469
	UART_SysSTAVar->TKS_Flag			=	G_StaV_St.TKS_Flag;        		//守时稳定标识
	UART_SysSTAVar->GPS_STA				=	G_StaV_St.GPS_STA;         		//GPS状态标示
	UART_SysSTAVar->GPS_Time			=	G_StaV_St.GPS_Time;           	//GPS时间
	UART_SysSTAVar->GPS_SatN			=	G_StaV_St.GPS_SatN;            	//GPS可见星数
	UART_SysSTAVar->GPS_longitude		=	(s32)(G_StaV_St.GPS_longitude * 10000000);     	//GPS经度
	UART_SysSTAVar->GPS_latitude		=	(s32)(G_StaV_St.GPS_latitude * 10000000);      	//GPS纬度
	UART_SysSTAVar->NATI_Time			=	G_StaV_St.NATI_Time;			//本机1PPS时戳
	UART_SysSTAVar->NATI_Time_Prog		=	G_StaV_St.NATI_Time_Prog;		//本机可编程脉冲时戳
	UART_SysSTAVar->Last_SYN_Time		=	G_StaV_St.Last_SYN_Time;		//上次同步时刻
	UART_SysSTAVar->Last_ACQ_s			=	G_StaV_St.Last_ACQ_s;			//上次同步测量误差s
	UART_SysSTAVar->Last_ACQ_ns			=	G_StaV_St.Last_ACQ_ns;			//上次同步测量误差ns
	UART_SysSTAVar->Last_ADJ_s			=	G_StaV_St.Last_ADJ_s;			//上次同步调整s
	UART_SysSTAVar->Last_ADJ_ns			=	G_StaV_St.Last_ADJ_ns;			//上次同步调整s
	UART_SysSTAVar->Auto_SYN			=	G_StaV_St.Auto_SYN;				//自同步使能
	UART_SysSTAVar->Auto_OCXOADJ		=	G_StaV_St.Auto_OCXOADJ;			//自调整晶振使能
	UART_SysSTAVar->Prog_Pul			=	G_StaV_St.Prog_Pul;				//可编程脉冲使能
	UART_SysSTAVar->Auto_KeepSIn		=	G_StaV_St.Auto_KeepSIn;			//自切换守时状态使能
	UART_SysSTAVar->Meas_Pul			=	G_StaV_St.Meas_Pul;				//测量功能使能
	UART_SysSTAVar->Prog_PerT			=	G_StaV_St.Prog_PerT;			//可编程脉冲周期
	UART_SysSTAVar->Messa_SYN			=	G_StaV_St.Messa_SYN;			//同步信息输出使能
	UART_SysSTAVar->Messa_1PPS			=	G_StaV_St.Messa_1PPS;			//本机秒脉冲信息使能
	UART_SysSTAVar->Messa_PGP			=	G_StaV_St.Messa_PGP;			//可编程脉冲信息使能
	UART_SysSTAVar->Messa_GPS			=	G_StaV_St.Messa_GPS;			//GPS 报文转发使能

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ sizeof(UART_SysSTAVarSt) + 5 ] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ sizeof(UART_SysSTAVarSt) + 6 ] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  发送错误命令帧
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
///  发送本机秒脉冲信息报文
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

	UART_NaviVar.TimeKeepSTA 	=	G_StaV_St.TimeKeepSTA ;         		//主循环状态机
	UART_NaviVar.TKS_Flag		=	G_StaV_St.TKS_Flag;        				//守时稳定标识
	UART_NaviVar.NATI_Time		= 	G_StaV_St.NATI_Time;					//本机1PPS时戳
	UART_NaviVar.GPS_STA		=	G_StaV_St.GPS_STA;         				//GPS状态标示
	UART_NaviVar.GPS_Time		= 	G_StaV_St.GPS_Time;               		//GPS时间
	UART_NaviVar.GPS_longitude	=	(s32)(G_StaV_St.GPS_longitude * 10000000); //GPS经度
	UART_NaviVar.GPS_latitude	= 	(s32)(G_StaV_St.GPS_latitude * 10000000); //GPS纬度

	*((UART_NaviVarSt *)(&(U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5]))) = UART_NaviVar ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ InfoSize + 5 ] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][ InfoSize + 6 ] = 0x0A ;
	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  发送近5次测量同步结果
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
///  发送同步对时结果
///
//////////////////////////////////////////////////////
void Send_Meas_Adj_Res(void)
{
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'a' ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] =  20 ;

	*((u32 *) & (U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5]))  = G_StaV_St.Last_SYN_Time ; //上次同步时刻
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
///  发送软件版本号
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
///  发芯片串号
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
///  转发GPS两种报文到PC
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
///  发送协议要求的数据帧，本机s
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

	SyFrameP->messData.uiYear		=	RTC_TIME_Var.QYear;	//年(4位十进制数值):>1900
	SyFrameP->messData.ucMonth		=	RTC_TIME_Var.Mon;	//月:1-12
	SyFrameP->messData.ucDay		=	RTC_TIME_Var.Dat ;	//天:1-31
	SyFrameP->messData.ucWeek		=	RTC_TIME_Var.Day ;	//week 1-7     add week
	SyFrameP->messData.ucHour		=	RTC_TIME_Var.Hor;	//24小时制:0-23
	SyFrameP->messData.ucMinute		=	RTC_TIME_Var.Min;	//分:0-59
	SyFrameP->messData.ucSecond		=	RTC_TIME_Var.Sec;	//秒:0-59
	//ms计数器<=1000
	SyFrameP->messData.usCounterms	=	0;

	/*
	坐标
	格式:N39.7172520000,E116.9242730000
	*/
	if(G_StaV_St.GPS_latitude > 0)
		SyFrameP->messData.NorS			=	'N';			//'N'/'n'北纬;'S'/'s'南纬
	else
		SyFrameP->messData.NorS			=	'S';

	if(G_StaV_St.GPS_longitude > 0)
		SyFrameP->messData.EorW			=	'E';			//'E'/'e'东经;'W'/'w'西经
	else
		SyFrameP->messData.EorW			=	'W';

	SyFrameP->messData.dLatitude	=	G_StaV_St.GPS_latitude;		//纬度  double float
	SyFrameP->messData.dLongitude	=	G_StaV_St.GPS_longitude;  	//经度

	SyFrameP->crc16Check			=	getCRC16((u8 *)SyFrameP, sizeof(SyFrame) - 2, crcTalbe) ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 5] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 6] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  发送协议要求的数据帧，GPSPPS
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

	SyFrameP->messData.uiYear		=	RTC_TIME_Var.QYear;	//年(4位十进制数值):>1900
	SyFrameP->messData.ucMonth		=	RTC_TIME_Var.Mon;	//月:1-12
	SyFrameP->messData.ucDay		=	RTC_TIME_Var.Dat ;	//天:1-31
	SyFrameP->messData.ucWeek		=	RTC_TIME_Var.Day ;	//week 1-7     add week
	SyFrameP->messData.ucHour		=	RTC_TIME_Var.Hor;	//24小时制:0-23
	SyFrameP->messData.ucMinute		=	RTC_TIME_Var.Min;	//分:0-59
	SyFrameP->messData.ucSecond		=	RTC_TIME_Var.Sec;	//秒:0-59
	//ms计数器<=1000
	SyFrameP->messData.usCounterms	=	0;

	/*
	坐标
	格式:N39.7172520000,E116.9242730000
	*/
	if(G_StaV_St.GPS_latitude > 0)
		SyFrameP->messData.NorS			=	'N';			//'N'/'n'北纬;'S'/'s'南纬
	else
		SyFrameP->messData.NorS			=	'S';

	if(G_StaV_St.GPS_longitude > 0)
		SyFrameP->messData.EorW			=	'E';			//'E'/'e'东经;'W'/'w'西经
	else
		SyFrameP->messData.EorW			=	'W';

	SyFrameP->messData.dLatitude	=	G_StaV_St.GPS_latitude;		//纬度  double float
	SyFrameP->messData.dLongitude	=	G_StaV_St.GPS_longitude;  	//经度

	SyFrameP->crc16Check			=	getCRC16((u8 *)SyFrameP, sizeof(SyFrame) - 2, crcTalbe) ;

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 5] = 0xFF ;
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 6] = 0x0A ;

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}
//////////////////////////////////////////////////////
///  发送协议要求的数据帧，测量脉冲时戳报文
///
//////////////////////////////////////////////////////
void Send_Sys_MeasPlu(u32 Mea_s, u32 Mea_ns)
{
	SyFrame	* SyFrameP ;
	RTC_TIME_Str 	RTC_TIME_Var ;

	Mea_ns					=	Mea_ns * 20 ;				//20ns单位处理成ns	捕获时戳ns
	RTC_TIME_Var.Unix_Time 	= 	Mea_s ;					//捕获时戳，s

	UnixT2Rtc(&RTC_TIME_Var);							//unix时间转为RTC时间

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][0] = '$' ;		//起始
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][1] = 'P' ;		//目的地址 ，P：上位机
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][2] = 'T' ;		//源地址	  ，T：守时模块
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][3] = 'R' ;		//命令域		R代表，合同要求报文
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][4] =  32 ; 		//数据域长度，32个字节

	SyFrameP	=	(SyFrame *)&U6_Sys_TxFrame[U6_Sys_Tx_InFrame][5];

	SyFrameP->fHead	=	0x7E ;																	//你的帧头
	SyFrameP->fType = 	0x01 ;																	//帧类型，待测脉冲时戳

	SyFrameP->messData.uiYear		=	RTC_TIME_Var.QYear;	//年(4位十进制数值):>1900
	SyFrameP->messData.ucMonth		=	RTC_TIME_Var.Mon;	//月:1-12
	SyFrameP->messData.ucDay		=	RTC_TIME_Var.Dat ;	//天:1-31
	SyFrameP->messData.ucWeek		=	RTC_TIME_Var.Day ;	//week 1-7     add week
	SyFrameP->messData.ucHour		=	RTC_TIME_Var.Hor;	//24小时制:0-23
	SyFrameP->messData.ucMinute		=	RTC_TIME_Var.Min;	//分:0-59
	SyFrameP->messData.ucSecond		=	RTC_TIME_Var.Sec;	//秒:0-59

	//ms计数器<=1000
	SyFrameP->messData.usCounterms	=	Mea_ns / 1000000;

	/*
	坐标
	格式:N39.7172520000,E116.9242730000															//坐标你不取，也在这里，若GPS无效，该区域是最后一次有效位置信息
	*/
	if(G_StaV_St.GPS_latitude > 0)
		SyFrameP->messData.NorS			=	'N';			//'N'/'n'北纬;'S'/'s'南纬
	else
		SyFrameP->messData.NorS			=	'S';

	if(G_StaV_St.GPS_longitude > 0)
		SyFrameP->messData.EorW			=	'E';			//'E'/'e'东经;'W'/'w'西经
	else
		SyFrameP->messData.EorW			=	'W';

	SyFrameP->messData.dLatitude	=	G_StaV_St.GPS_latitude;		//纬度
	SyFrameP->messData.dLongitude	=	G_StaV_St.GPS_longitude;  	//经度

	SyFrameP->crc16Check			=	getCRC16((u8 *)SyFrameP, sizeof(SyFrame) - 2, crcTalbe) ;	//你的CRC16校验

	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 5] = 0xFF ;								//我的BCC校验，此时为0xFF，在发射处理时填充
	U6_Sys_TxFrame[U6_Sys_Tx_InFrame][sizeof(SyFrame) + 6] = 0x0A ;								//我的结束字节

	U6_Sys_Tx_InFrame++;
	U6_Sys_Tx_InFrame %= 20;
}

///////////////////////////////////////////////////////////
/////处理PC来的串口报文事务
extern u8  Err_Temp ;				//临时错误标识
extern u8  ProgP_SYN_Flag;			//同步脉冲需要重新同步的标志位
extern SYN_Info	Last_SYN_Table[5];	//最近5次同步信息记录表
extern u8		Next_SYN_Item;		//指向一个最老的同步或测量信息

char ProcPC_CMD(u8 New_CMD_Flag, u8 * New_Data )
{
	u32		Prog_PerT_temp ;			//可编程周期脉冲设置时，需要用到的临时变量

	if( New_CMD_Flag == 'T')			//查询状态
	{
		New_CMD_Flag = 0 ;
		//上位机查询状态
		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'O')							//设置恒温晶振
	{
		New_CMD_Flag = 0 ;

		if( *((u16*)(&New_Data[0])) <= 1023 && *((u16*)(&New_Data[0])) >= 10 )
		{
			G_StaV_St.DigPot = *((u16*)(&New_Data[0])) ;
			DIGPOT_Write(G_StaV_St.DigPot);
			Send_STA_Sys();                     			//考虑到这个指令用的不多,所以直接回复状态,
		}
		else
		{
			Err_Temp = 2;
			Send_Err('O', &Err_Temp, 2);          			//超过最大时延增量,报错1
		}
	}
	else if(New_CMD_Flag == 'B')							//压控自调整,根据，每2分钟的漂移误差，决定，是否更改恒温晶振电压
	{
		//要满足，已经开机大于40min，GPS脉冲稳定 这些条件
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Auto_OCXOADJ != 0)						//反转压控自调整
			G_StaV_St.Auto_OCXOADJ	= 0;
		else
			G_StaV_St.Auto_OCXOADJ	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'A')							//自同步，当GPS锁定，且秒脉冲有效，可自动完成时钟同步
	{
		//可在初始状态或同步状态下自动同步操作。
		New_CMD_Flag = 0 ;									//在守时状态下，默认GPS模块断电

		//在守时状态下，若GPS被手动打开，可完成自动测量，但不同步！只测量漂移误差。
		if(G_StaV_St.Auto_SYN != 0)		//反转自同步			//若自动同步功能关闭，但GPS打开且信号稳定，每2min，测量漂移误差1次
			G_StaV_St.Auto_SYN	= 0;
		else
			G_StaV_St.Auto_SYN	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'G')		//GPS电源			//切换状态时，会根据状态，确定GPS电源状态。然后，可手动关闭或打开,打开后，无论如何会完成误差测量
	{
		New_CMD_Flag = 0 ;
		/*
				此处功能，迁移至IO口，不再串口中处理状态，否则，逻辑复杂
				if(GPS_Power != 0)				//反转GPS电源
				{
					GPS_Power			= 	0;
					G_StaV_St.GPS_STA	=	GPS_RNSS_OFF ;
				}
				else
				{
					GPS_Power			= 	1;
					G_StaV_St.GPS_STA	=	GPS_RNSS_V ;
					G_StaV_St.TKS_Flag 	= 	0 ;						//打开GPS后重新锁定
					TKPSTALED			= 	LED_Flash_100Ms;
				}

				GPS_Pow(GPS_Power);
				Send_STA_Sys();
		*/
		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'C')		//同步信息输出
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_SYN != 0)	//反转同步信息输出
			G_StaV_St.Messa_SYN	= 0;
		else
			G_StaV_St.Messa_SYN	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'P')		//本机秒脉冲信息输出
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_1PPS != 0)	//反转本机秒脉冲信息输出
			G_StaV_St.Messa_1PPS = 0;
		else
			G_StaV_St.Messa_1PPS = 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'W')		//可编程脉冲信息输出
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_PGP != 0)	//反转可编程脉冲信息输出
			G_StaV_St.Messa_PGP	= 0;
		else
			G_StaV_St.Messa_PGP	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'V')		//可编程脉冲输出
	{
		New_CMD_Flag = 0 ;

		Prog_PerT_temp = *((u32*)(&New_Data[0])) ;					//把传递来的脉冲宽度取出来

		if( G_StaV_St.Prog_PerT + 1 != MS2ON20NS(Prog_PerT_temp)	//和当前的不一样
				&&	Prog_PerT_temp % 100 	== 0							//是0.1s的整倍数
				&&	Prog_PerT_temp		 	<= 60000						//小于等于60s
				&&	Prog_PerT_temp		 	>=  1000)						//大于等于1s
		{
			G_StaV_St.Prog_PerT = MS2ON20NS(Prog_PerT_temp) - 1;

			ProgP_SYN_Flag 	= 1 ;									//更改了周期参数，可以准备重新同步可编程周期脉冲
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
	else if(New_CMD_Flag == 'N')		//自切换守时状态使能
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Auto_KeepSIn != 0)	//反转自切换守时状态使能
			G_StaV_St.Auto_KeepSIn = 0;
		else
			G_StaV_St.Auto_KeepSIn = 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'M')		//脉冲测量功能使能
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Meas_Pul	!= 0)	//反转脉冲测量功能使能
			G_StaV_St.Meas_Pul	 = 0;
		else
			G_StaV_St.Meas_Pul	 = 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'Y')		//GPS报文转发
	{
		New_CMD_Flag = 0 ;

		if(G_StaV_St.Messa_GPS != 0)	//打开或关闭GPS报文转发
			G_StaV_St.Messa_GPS	= 0;
		else
			G_StaV_St.Messa_GPS	= 1;

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'L')		//最近5次同步或测量结果
	{

		New_CMD_Flag = 0 ;
		Send_Last_SYN_Table(&(Last_SYN_Table[0]), Next_SYN_Item);

	}
	else if(New_CMD_Flag == 'Z')		//恢复出厂设置
	{
		New_CMD_Flag = 0 ;

		G_StaV_St.DigPot		= 513;             		//数字电位器值，默认为513
		G_StaV_St.Auto_SYN		= 1;					//自同步使能
		G_StaV_St.Auto_OCXOADJ	= 0;					//自调整晶振使能
		G_StaV_St.Prog_Pul		= 0;					//可编程脉冲使能
		G_StaV_St.Auto_KeepSIn	= 0;					//自切换守时状态使能
		G_StaV_St.Meas_Pul		= 1;					//测量功能使能
		G_StaV_St.Prog_PerT		= MS2ON20NS(1600) - 1;	//可编程脉冲周期
		G_StaV_St.Messa_SYN		= 1;					//同步信息输出使能
		G_StaV_St.Messa_1PPS	= 1;					//本机秒脉冲信息使能
		G_StaV_St.Messa_PGP		= 0;					//可编程脉冲信息使能
		G_StaV_St.Messa_GPS		= 0;					//GPS 报文转发使能

		Set_Para_IIC(&G_StaV_St);

		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'K')		//保存临时参数设置
	{
		New_CMD_Flag = 0 ;
		//Set_Para(&G_StaV_St);
		Set_Para_IIC(&G_StaV_St);
		Send_STA_Sys();
	}
	else if(New_CMD_Flag == 'U')		//获取版本号
	{
		New_CMD_Flag = 0 ;
		Send_SoftVer();
	}
	else if(New_CMD_Flag == 'H')		//获取版本号
	{
		New_CMD_Flag = 0 ;
		Send_SIGID();
	}

	//缺少‘S’切换状态  ‘Q’强制同步

	return New_CMD_Flag;
}
