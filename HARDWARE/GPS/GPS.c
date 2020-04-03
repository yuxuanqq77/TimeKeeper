#include "sys.h"
#include "GPS.h"
#include "timer.h"
#include "string.h"
#include "usart.h"

extern u32	GPS_A_V_TimerOve ;
extern u32	GPS_A_V_TimerOve50ms ;
extern u8	GPSD_Send_Flag;

//串口接收数据缓冲区，DMA自动填充，循环缓冲
__align(4) u8  U1_GPS_RxBuf[UARTRXBUFLEN_U1] ;
u16 U1_GPS_Rx_ProcPos = 0;          //指向还没有处理的字节
u16 U1_GPS_Rx_GetPos  = 0;          //指向还没有被填充的字节，在接收处理函数中更新

//解析出来的有效帧
__align(4) u8  U1_GPS_RxFrame[20][512] ;
u16 U1_GPS_Rx_ProcFrame = 0;        //指向还没有处理的有效帧
u16 U1_GPS_Rx_GetFrame  = 0;        //指向还没有被解析出来的空帧


//////////////////////////////////////
////  UART1 RX 使用的DMA
////  DMA2 Stream2 Ch4 初始化
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

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; 					//DMA2时钟使能

	DMA_Streamx->CR &= ~(DMA_SxCR_EN); 						//关闭DMA传输

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

	DMA_Streamx->PAR  = (u32) & (USART1->DR);		  			//DMA外设地址
	DMA_Streamx->M0AR = (u32)U1_GPS_RxBuf ;		  			//DMA 存储器0地址
	DMA_Streamx->NDTR = UARTRXBUFLEN_U1;		        	//
	DMA_Streamx->CR   = 0;			                		//先全部复位CR寄存器值

	DMA_Streamx->CR  &= ~DMA_SxCR_DIR;		    			//外设到存储器模式
	DMA_Streamx->CR  |= DMA_SxCR_CIRC;	    				//循环模式
	DMA_Streamx->CR  &= ~DMA_SxCR_PINC;		  				//外设非增量模式
	DMA_Streamx->CR  |= DMA_SxCR_MINC;		  				//存储器增量模式
	DMA_Streamx->CR  &= ~DMA_SxCR_PSIZE;		  				//外设数据长度:8位
	DMA_Streamx->CR  &= ~DMA_SxCR_MSIZE;		  				//存储器数据长度:8位
	DMA_Streamx->CR  &= ~DMA_SxCR_PL;		    			//低优先级
	DMA_Streamx->CR  &= ~DMA_SxCR_PBURST;					//外设突发单次传输
	DMA_Streamx->CR  &= ~DMA_SxCR_MBURST;					//存储器突发单次传输

	DMA_Streamx->CR  |= (u32)chx << 25;     //通道选择
	DMA_Streamx->CR  &= ~DMA_SxCR_TCIE ;    //不要中断
	DMA_Streamx->CR  |= DMA_SxCR_EN;        //打开DMA
}


//////////////////////////////////////////////////////////////////////////////////
//// UART1向GPS的串口初始化
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
void UART1_GPS_init(u32 pclk2, u32 bound)
{
	float temp;

	u16 mantissa;
	u16 fraction;
	UART1_RX_DMA_Config();                  			//配置循环接收DMA，并启动
	temp = (float)(pclk2 * 1000000) / (bound * 16); 	//得到USARTDIV@OVER8=0
	mantissa = temp;				       				//得到整数部分
	fraction = (temp - mantissa) * 16;            		//得到小数部分@OVER8=0
	mantissa <<= 4;
	mantissa += fraction;
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN ;   	//使能GPIOA口时钟
	RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;  	//使能串口1时钟
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;  	//复位串口1
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
	GPIO_Set(GPIOA, PIN9 | PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_PU); //PA10,PA9,复用功能,上拉输出
	GPIO_AF_Set(GPIOA, 10, 7);					//PA9, AF7
	GPIO_AF_Set(GPIOA, 9, 7);  					//PA10,AF7
	//波特率设置
	USART1->BRR = mantissa; 	    			//波特率设置
	USART1->CR1 &= ~(1 << 15); 	    			//设置OVER8=0
	USART1->CR3 |= USART_CR3_DMAT;       		//接收DMA使能
	USART1->CR1 |= USART_CR1_TE;  	     		//串口发送使能
	mantissa = USART1->DR ;              		//清空一下DR寄存器
	USART1->CR3 |= USART_CR3_DMAR;       		//发送DMA使能
	USART1->CR1 |= USART_CR1_RE;  	     		//串口接收使能
	USART1->CR1 |= USART_CR1_UE;  	     		//串口使能
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

extern SYS_GSV		G_StaV_St ;

u32 UART1_GPS_OveTimeS ;        //超时计数器变量
u8  UART1_GPS_OveTime50MS;     	//

//异或和校验计算，协议只定义从'$'到BCC字节之间的所有待传送字节异或
u8 CheckBccGPS(u8 * CheckBuf, u16 CheckLen)
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
u16 Proc_RxByte_GPS(void)
{
	static u8  Uart1_GPS_STA  = UART_IDLE ;
	static u16 RxProPos       = 0 ;

	static u8  UART1_GPS_OveTime_Flag = 1 ;										//超时标志位 1超时或空闲，0启动超时判断

	u16 i = 0 ;
	u8  ReaD ;
	u8  BCCtemp1, BCCtemp2;

	//串口超时，从收到‘$’开始计时，超过1s认为超时，恢复到空闲状态
	if( UART1_GPS_OveTime_Flag != 1 )
	{
		if( CheckTimerOve(UART1_GPS_OveTimeS, UART1_GPS_OveTime50MS) == 1
				&&  Uart1_GPS_STA != UART_IDLE )
		{
			Uart1_GPS_STA = UART_IDLE ;
			UART1_GPS_OveTime_Flag = 1;
		}
	}

	//取出现在接收到哪个字节位置
	//U1_GPS_Rx_GetPos始终指向一个DMA还没有用的位置，空数据
	//U1_GPS_Rx_ProcPos始终指向一个还没有处理的位置
	//两个变量一样，说明，没有需要处理的字节，这里没有考虑突然来了很多字节
	//循环溢出的错误。
	//只是认为接收缓冲区足够大，大到不可能出现循环溢出错误

	U1_GPS_Rx_GetPos = ( UARTRXBUFLEN_U1 - DMA2_Stream2->NDTR ) ;

	//最多处理20个字节，避免时间太长，影响主循环
	while(i < 20)
	{
		if(U1_GPS_Rx_ProcPos == U1_GPS_Rx_GetPos)     							//没有要处理数据
			break;
		else
		{
			ReaD = U1_GPS_RxBuf[U1_GPS_Rx_ProcPos] ;    						//有要处理的字节，取出

			U1_GPS_Rx_ProcPos++;                        						//取出后循环+1
			U1_GPS_Rx_ProcPos %= UARTRXBUFLEN_U1;

			switch(Uart1_GPS_STA)
			{
				case UART_IDLE:                           						//空闲状态，只有收到$，才会跳出

					if(ReaD == '$')
					{
						RxProPos = 0 ;
						U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][RxProPos] = ReaD ;
						RxProPos++;

						UART1_GPS_OveTime_Flag = 0 ;                			//开始超时计数

						UART1_GPS_OveTimeS     = G_StaV_St.RunTime + 1 ;        	//设置超时时间1s
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

						//在这里，把报文的长度替代了‘$’
						U1_GPS_RxFrame[U1_GPS_Rx_GetFrame][0] = RxProPos ;
						//这里更新有效帧
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
	//返回还有多少个字节没有处理
	return (U1_GPS_Rx_GetPos + UARTRXBUFLEN_U1 - U1_GPS_Rx_ProcPos) % UARTRXBUFLEN_U1 ;
}
//

//////////////////////////////////////////////////////
///  主程序中取出一个最老的有效帧，命令及数据
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
//取数据域，不要'.'
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

	//TempPos指向需要取出来的第一个字节
	if( TempPos >= 200)
		return 0xFF ;//返回错误

	for(i = 0; i < 50; i++)
	{
		if(StrIn[TempPos + i] == ',' || StrIn[TempPos + i] == '*' || StrIn[TempPos + i] == '.')
			break;

		OutStr[i] = StrIn[TempPos + i];
	}

	if(i >= 50)
		return 0xFF ;//返回错误

	OutStr[i] = 0 ;//字符串以NULL结束

	return i;
}
//取数据域，要'.'
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

	//TempPos指向需要取出来的第一个字节
	if( TempPos >= 200)
		return 0xFF ;//返回错误

	for(i = 0; i < 50; i++)
	{
		if(StrIn[TempPos + i] == ',' || StrIn[TempPos + i] == '*')
			break;

		OutStr[i] = StrIn[TempPos + i];
	}

	if(i >= 50)
		return 0xFF ;//返回错误

	OutStr[i] = 0 ;//字符串以NULL结束

	return i;
}
//
//处理RMC、GSV报文
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

		if(Region_Str[0] == 'A')                  						//收到定位有效报文
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

			G_StaV_St.GPS_Time = RTC_TIME_Var.Unix_Time + 28800;		//这里加上了28800,就是8个小时,折算到北京时间

			if( G_StaV_St.GPS_STA != GPS_RNSS_A )                		//更新之前看看之前的状态是不是有效
			{
				//如果不是有效,就向PC机更新一下状态
				G_StaV_St.GPS_STA  = GPS_RNSS_A;
				Send_STA_Sys();
			}

			//取出经纬度数据

			if(	Get_Region_Pos_wp(RNSS_GPSData, 3, Region_Str) == 9)			//取出纬度数据	GPS15L协议，小数点后4位
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
			else if( Get_Region_Pos_wp(RNSS_GPSData, 3, Region_Str) == 10)		//取出纬度数据 UBlox协议，小数点后5位
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
				G_StaV_St.GPS_latitude =  G_StaV_St.GPS_latitude ;				//北纬+
			else
				G_StaV_St.GPS_latitude = -G_StaV_St.GPS_latitude;				//南纬-

			if(	Get_Region_Pos_wp(RNSS_GPSData, 5, Region_Str) == 10)			//取出经度数据	GPS15L协议，小数点后4位
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
			else if( Get_Region_Pos_wp(RNSS_GPSData, 5, Region_Str) == 11)	//取出经度数据	UBlox协议，小数点后5位
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
				G_StaV_St.GPS_longitude =  G_StaV_St.GPS_longitude;	//东经+
			else
				G_StaV_St.GPS_longitude = -G_StaV_St.GPS_longitude;	//西经-

			if(G_StaV_St.Messa_1PPS == 1)
				GPSD_Send_Flag	= 1 ;	//允许发射GPS报文给上位机
			else
				GPSD_Send_Flag	= 0 ;

			GPS_A_V_TimerOve      = G_StaV_St.RunTime + 2;    		//设置GPS报文更新超时,如果两秒之内
			GPS_A_V_TimerOve50ms  = G_StaV_St.RunTime_50ms;   		//没有更新,就把GPS设置为无效.

		}
		else                                      					//收到'V'报文
		{
			if(G_StaV_St.GPS_STA != GPS_RNSS_V ) 					//PotA 曾经是关机或者有效,现在没有信号了.
			{
				//更改GPS状态,而后上报PC机
				G_StaV_St.GPS_STA = GPS_RNSS_V;
				Send_STA_Sys();
			}
		}
	}

	//处理GSV报文,其实只把可见星数取出来
	if(strcmp("$GPGSV", (char *)Region_Str) == 0 )
	{
		if(Get_Region_Pos(RNSS_GPSData, 3, Region_Str) != 2)
			return;

		G_StaV_St.GPS_SatN = (Region_Str[0] - '0') * 10 + (Region_Str[1] - '0') ;
	}

}
//////////////////////////////
////GPS报文的顶层处理函数
////
void Prorc_GPS_Top(void)
{
	u8	New_GPS_Flag = 0;				//新的GPS命令，来自GPS
	u8	New_GPS_Data[512];				//来自GPS的报文
	u8	GPSLenTemp;						//GPS报文的长度

	//解码GPS报文
	if( G_StaV_St.GPS_STA != GPS_RNSS_OFF )											//这里处理GPS相关的报文，如果GPS时关机的，就不处理了
	{
		Proc_RxByte_GPS();															//处理GPS原始报文
		New_GPS_Flag 	= Proc_RxCMD_GPS(New_GPS_Data);								//取出一个最老的报文

		if( New_GPS_Flag	!=	0 )													//如果有新的GPS报文到来
		{
			GPSLenTemp 		= 	New_GPS_Data[0] ;
			New_GPS_Data[0]	=	'$' ;
			Prorc_RNSS_GPS_RMC_GSV( New_GPS_Data );									//处理GPS报文，包括状态，时间，星数等等信息
			New_GPS_Data[0] = GPSLenTemp ;

			if( G_StaV_St.Messa_GPS == 1 )											//如果打开GPS转发使能
			{
				if(New_GPS_Flag == 'C')												//RMC转发全文
				{
					//转发GPS报文
					Send_GPS_C_V(New_GPS_Data);
				}
				else if( New_GPS_Flag == 'V' && New_GPS_Data[9] == '1')				//GSV只转发第1条
				{
					//转发GPS报文
					//Send_GPS_C_V(New_GPS_Data);									//不再转发GSV报文
				}
			}

			New_GPS_Flag = 0 ;
		}
	}
}

//GPS电源控制初始化
void GPS_Pow_init(void)
{
	RCC->AHB1ENR    |=  RCC_AHB1ENR_GPIOCEN;//使能PORTC时钟
	POWGPS = 0 ;
	GPIO_Set(GPIOC, PIN5, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); //

	RCC->AHB1ENR    |=  RCC_AHB1ENR_GPIOCEN;//使能PORTC时钟  GPS_P_C
	GPIO_Set(GPIOC, PIN9, GPIO_MODE_IN, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE);  //

	G_StaV_St.GPS_STA 	=	GPS_RNSS_OFF ;
	GPS_Pow(0);
}

//更改GPS电源
void GPS_Pow(u8 OnOff)
{
	if(OnOff)     POWGPS = 1 ;
	else          POWGPS = 0 ;
}
///////////////////////////////////////////////////////////
///这是个新函数,功能是检测外部GPS电源控制IO,刷新GPS模块的电源
void GPS_Pow_Re(void)
{
	static u8  	GPS_Power = 0 ;		//当前的GPS电源是否打开
	static u16	GPS_V_Timer = 0;	//GPS电源打开情况下,处于无效状态的时间,用于自动重启GPS

	u8	GPSPow_PinV;

	GPSPow_PinV	= GPS_P_C ; 		//拿到外部控制信号 1：关闭 0：打开

	if( GPSPow_PinV == 0)			//外部要求打开GPS
	{
		if(GPS_Power == 1)			//如果现在已经打开
		{
		}
		else						//现在没有打开
		{
			G_StaV_St.GPS_STA 	=	GPS_RNSS_V ;
			GPS_Power 			= 1;
			GPS_Pow(GPS_Power);
			GPS_V_Timer 		= 	0 ;
			Send_STA_Sys();
		}
	}
	else							//外部要求关闭电源
	{
		if(GPS_Power == 1)			//如果现在已经打开
		{
			G_StaV_St.GPS_STA 	=	GPS_RNSS_OFF ;
			GPS_Power 			= 0;
			GPS_Pow(GPS_Power);
			GPS_V_Timer 		= 	0 ;
			Send_STA_Sys();
		}
		else						//现在没有打开
		{
		}
	}

	//以下代码，完成GPS的复位，当GPS，600s没有定位后，关掉GPS电源，3s后，打开GPS电源
	if(G_StaV_St.GPS_STA  == GPS_RNSS_V)
	{
		GPS_V_Timer++;				//只有在GPS无效这个状态下，才去积累时间

		if(GPS_V_Timer >= 600 && GPS_V_Timer < 800)
		{
			GPS_V_Timer = 800 ;		//设置800，跳到待开机状态，这里没有取碰GPS_Power，视为当前的GPS还在开机状态
			GPS_Pow(0);				//关闭GPS电源
		}

		if(GPS_V_Timer > 802)
		{
			GPS_V_Timer = 0 ;		//可以重新打开GPS电源 801、802、803，合计3s，这中间如果关闭gps，就会去其他状态。安全。
			GPS_Pow(1);				//打开GPS电源
		}
	}
	else
	{
		GPS_V_Timer = 0 ;			//有效、关机，都会到这里清除GPS_V_Timer
	}
}


