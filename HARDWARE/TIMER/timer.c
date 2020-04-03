#include "timer.h"

extern SYS_GSV		G_StaV_St ;

extern u32 			PulW_NAT1P;        				//第1脉宽，以20ns为单位
extern u32 			PerTimer;

extern u8  			AcqTimeResFlag ;               	//捕获测量标志位
extern u32 			AcqTime20ns ;                 	//捕获的时延值，以20ns为单位
extern u32 			AcqTimeS ;
extern u32 			AcqGPSTime;

extern s32 			Correct_m ;
extern u8  			AdjTimeResFlag ;

extern u8  			NaviD_Send_Flag ;

extern u8			exPulMeasFlag;
extern u32			exPul_TimerOve;				//外部脉冲保护间隔设置
extern u8 			exPul_TimerOve50ms;
extern u32			exPulMeas_ns;
extern u32			exPulMeas_s;
//
/////////////////////////////////////////////////////////
///   主计数器TIM2初始化，按照信号周期、第一脉冲宽度、
///   同步通道。设置TIM2。完成守时、第一脉冲生成、测
///   量、同步等功能  输入50MHz @20ns
/////////////////////////////////////////////////////////
void TIM2_KEEP_Init(u32 PerTimerI, u32 PulW_NAT1PI) //
{
	RCC->APB1ENR  |=  RCC_APB1ENR_TIM2EN;     				//打开TIM2电源
	RCC->APB1RSTR |=  RCC_APB1RSTR_TIM2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;     				//使能PORTB时钟  第1脉冲信号
	GPIO_Set(GPIOB, PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);
	GPIO_AF_Set(GPIOB, 10, 1);	  							//PB10,AF1

	TIM2->PSC = 0 ;			        						//20ns 50MHz
	TIM2->ARR = PerTimerI ;									//周期

	TIM2->CCMR2 |=  TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;  	//第1脉冲信号,生成 T2_C3  6<<4 PWM模式1
	TIM2->CCMR2 |=  TIM_CCMR2_OC3PE; 	                  	//1<<3
	TIM2->CCER  |=  TIM_CCER_CC3E;   	                  	//1<<8
	TIM2->CCER  &= ~TIM_CCER_CC3P;   	                  	//0<<9	*这里可能要反向
//	TIM2->CCER  |=  TIM_CCER_CC3P;

	TIM2->CCR3   =  PulW_NAT1PI;                          	//这里不 “-1”,设置第一脉冲宽度
	TIM2->CR1   |=  TIM_CR1_ARPE;   	                  	//1<<7

	//捕获端口初始化
	RCC->AHB1ENR  |=  RCC_AHB1ENR_GPIOAEN;      			//使能PORTA时钟
	GPIO_Set(GPIOA, PIN1, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);
	GPIO_AF_Set(GPIOA, 1, 1);	                    		//PA1,AF1

	TIM2->CCMR1 |= TIM_CCMR1_CC2S_0 ;   					//CC2S[1:0] CC2输入，映射到TI2
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S_1 ;
	TIM2->CCMR1 |= 0 << 10 ;              					//IC2PSC[1:0]
	TIM2->CCMR1 |= 0 << 12 ;             					//IC2F

	TIM2->CCER &= ~(TIM_CCER_CC2NP);   	  					//CC2NP
	TIM2->CCER &= ~(TIM_CCER_CC2P);   	  					//CC2P  上升沿捕获

	TIM2->SR  &= ~(TIM_SR_CC2IF);        					//清除中断标志位
	TIM2->DIER &= ~(TIM_DIER_CC2IE);      					//CC2IE 不打开中断
	TIM2->CCER &= ~(TIM_CCER_CC2E);   	  					//CC2E  不打开捕获

//////////////////////////////////////////////////////
////在这里初始化外部脉冲测量IO *待调试验证
	RCC->AHB1ENR  |=  RCC_AHB1ENR_GPIOBEN;      			//使能PORTB时钟
	GPIO_Set(GPIOB, PIN11, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);
	GPIO_AF_Set(GPIOB, 11, 1);	                    		//PB11,AF1

	TIM2->CCMR2 |= TIM_CCMR2_CC4S_0 ;   					//CC4S[1:0] CC4输入，映射到TI4
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S_1 ;
//	TIM2->CCMR2 |= 0 << 10 ;              					//IC4PSC[1:0]
//	TIM2->CCMR2 |= 0 << 12 ;             					//IC4F

	TIM2->CCER &= ~(TIM_CCER_CC4NP);   	  					//CC4NP
	TIM2->CCER &= ~(TIM_CCER_CC4P);   	  					//CC4P  上升沿捕获

	TIM2->SR   &= ~(TIM_SR_CC4IF);        					//清除中断标志位
	TIM2->DIER &= ~(TIM_DIER_CC4IE);      					//CC4IE 不打开中断
	TIM2->CCER &= ~(TIM_CCER_CC4E);   	  					//CC4E  不打开捕获

////在这里初始化外部脉冲测量IO  *待调试验证
////////////////////////////////////////////////////

	//设置半周期调整通道
	TIM2->CCR1   = (PerTimerI + 1) / 2;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1CE) ;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M)  ;
	TIM2->CCMR1 |=  TIM_CCMR1_OC1PE  ;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1FE) ;
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S) ;

	MY_NVIC_Init(0, 1, TIM2_IRQn, 2);						//抢占0，子优先级1，组2

	TIM2->SR   &= ~(TIM_SR_UIF);          					//清中断标志
	TIM2->DIER |=  TIM_DIER_UIE;          					//开中断

	TIM2->EGR |=  TIM_EGR_UG;       						//UG置1
	TIM2->CR1 |=  TIM_CR1_CEN;      						//启动定时器
}
//设置第一脉冲宽度
void Set_PulW_NAT1P(u32 PulW_NAT1PI)
{
	PulW_NAT1P	=  PulW_NAT1PI;              				//第1脉宽，以20ns为单位，默认为1.25ms
	TIM2->CCR3	=  PulW_NAT1PI;              				//这里不 “-1”,设置第一脉冲宽度
}
//

//打开外部脉冲测量
void TIM2_ExPul_Meas_Start(void)
{
	TIM2->CCER  &= ~(TIM_CCER_CC4NP);   	//CC4NP
	TIM2->CCER  &= ~(TIM_CCER_CC4P);   	  	//CC4P  上升沿捕获

	TIM2->SR    &= ~(TIM_SR_CC4IF);        	//清除中断标志位
	TIM2->DIER  |=  TIM_DIER_CC4IE;       	//CC4IE 打开中断
	TIM2->CCER  |=  TIM_CCER_CC4E;   	    //CC4E  打开捕获

	exPulMeasFlag    	= EXPUL_STA_WAT ;
}

void TIM2_ExPul_Meas_Stop(void)
{
	TIM2->CCER  &= ~(TIM_CCER_CC4NP);   	//CC4NP
	TIM2->CCER  &= ~(TIM_CCER_CC4P);   	  	//CC4P  上升沿捕获

	TIM2->SR    &= ~(TIM_SR_CC4IF);        	//清除中断标志位

	TIM2->CCER  &= ~TIM_CCER_CC4E;   	    //CC4E  打开捕获
	TIM2->DIER  &= ~TIM_DIER_CC4IE;       	//CC4IE 打开中断

	exPulMeasFlag    	= EXPUL_STA_OFF ;
}

///////////////////////////////////////////////////
///    打开一次测量序列，并设置超时时间
///    根据同步通道设置，设置信号捕获的IO口
///////////////////////////////////////////////////
u8  Meas_OVETime = 0;

void TIM2_Meas_Start(void)
{
	TIM2->CCER  &= ~(TIM_CCER_CC2NP);   	//CC2NP
	TIM2->CCER  &= ~(TIM_CCER_CC2P);   	  	//CC2P  上升沿捕获

	TIM2->SR    &= ~(TIM_SR_CC2IF);        	//清除中断标志位
	TIM2->DIER  |=  TIM_DIER_CC2IE;       	//CC2IE 打开中断
	TIM2->CCER  |=  TIM_CCER_CC2E;   	    //CC2E  打开捕获

	//设置超时时间50*30=1.5s 考虑到主控单片机在下一个
	//GPS秒脉冲来之前1s内发送同步或测量指令，因此该超时设置为不到2s
	Meas_OVETime = 30 ;
}
//

///////////////////////////////////////////////////
///    打开一次调整序列，并设置超时时间
///    根据同步通道设置，设置信号捕获的IO口
///////////////////////////////////////////////////
u16 Adj_OVETime = 0 ;

void TIM2_Adj_Start(void)
{
	TIM2->CCER  &=	~(TIM_CCER_CC1NP);   	   	//CC1NP
	TIM2->CCER  &=	~(TIM_CCER_CC1P);   	   	//CC1P
	TIM2->CCMR1 |=	 0x06 << 4;                	//PWM1,OC1M  = 110
	TIM2->SR    &=	~(TIM_SR_CC1IF);         	//清除中断标志位
	TIM2->DIER  |=	  TIM_DIER_CC1IE;     		//CC1IE 打开中断

	//设置超时时间 50*30=1.5s后超时
	Adj_OVETime = 30 ;
}
//

//软件带来的调整量修正，从示波器观察得到
s32 Adj_SoftC = 13 ;

//定时器2中断服务程序
//这是这个程序的核心
//根据得到测量值
//根据测量值调整TCNT
//这个地方逻辑非常复杂,都是在边界上走,所以小心验证，有个visio，在工程文件夹里！
void TIM2_IRQHandler(void)
{
	//只可能有3种中断产生
	//1.捕获同步脉冲
	//2.本机秒脉冲更新中断
	//3.半周期中断，完成TCNT调整
	//4.外部待测脉冲捕获中断，用于测量外部脉冲时戳

	//如果是 捕获中断 和 更新中断
	if(TIM2->SR & TIM_SR_CC2IF)	//捕获中断
	{
		if(TIM2->DIER & TIM_DIER_CC2IE)
		{
			AcqTime20ns			= TIM2->CCR2 ;            				//把捕获的结果取出来
			AcqTimeS			= G_StaV_St.NATI_Time ;

			AcqTimeResFlag    	= 1 ;                     				//设置标志位，让主程序知道捕获完成

			TIM2->CCER  &= ~(TIM_CCER_CC2E);   	        				//CC2E  关闭捕获
			TIM2->DIER  &= ~(TIM_DIER_CC2IE);            				//CC2IE 关闭中断
			TIM2->SR    &= ~(TIM_SR_CC2IF);              				//清除中断标志位

			if( AcqTime20ns < 50000 )									//假如捕获的结果小于1ms，这里有可能需要对捕获的s变量做补偿
			{
				if(TIM2->SR	& TIM_SR_UIF)								//假如已经发生了更新中断，并且没有处理
				{
					AcqTimeS++ ;

					//本机秒脉冲
//这个地方不能更新时间，可能会影响到 外部脉冲捕获中断
//					G_StaV_St.NATI_Time++ ;								//顺带手把本机秒脉冲也给更新了算了
//					TIM2->SR    &=	~(TIM_SR_UIF);            			//清除中断标志位
				}
				else
				{
					//跑到这里，是因为，捕获的虽然比较小，但是，更新已经响应过了，取到的ns、s都是正确的
				}
			}
		}
		else
		{
			//正常情况不会跑到这里，但是真跑这了，还是把中断清了比较安全
			TIM2->CCER  &= ~(TIM_CCER_CC2E);   	        				//CC2E  关闭捕获
			TIM2->DIER  &= ~(TIM_DIER_CC2IE);            				//CC2IE 关闭中断
			TIM2->SR    &= ~(TIM_SR_CC2IF);              				//清除中断标志位
		}
	}
	else if(TIM2->SR & TIM_SR_CC4IF)									//4.外部待测脉冲捕获中断，用于测量外部脉冲时戳
	{
		if(TIM2->DIER & TIM_DIER_CC4IE)
		{
			exPulMeas_ns		= TIM2->CCR4 ;            				//把捕获的结果取出来
			exPulMeas_s			= G_StaV_St.NATI_Time ;

			TIM2->CCER  &= ~(TIM_CCER_CC4E);   	        				//CC4E  关闭捕获
			TIM2->DIER  &= ~(TIM_DIER_CC4IE);            				//CC4IE 关闭中断
			TIM2->SR    &= ~(TIM_SR_CC4IF);              				//清除中断标志位 关掉是为了实现间隔保护

			if( exPulMeas_ns < 50000 )									//假如捕获的结果小于1ms，这里有可能需要对捕获的s变量做补偿
			{
				if(TIM2->SR	& TIM_SR_UIF)								//假如已经发生了更新中断，并且没有处理
				{
					exPulMeas_s++ ;
//这个地方不能更新时间，可能会影响到同步捕获中断
//					G_StaV_St.NATI_Time++ ;								//顺带手把本机秒脉冲也给更新了算了
//					TIM2->SR    &=	~(TIM_SR_UIF);            			//清除中断标志位
				}
				else
				{
					//跑到这里，是因为，捕获的虽然比较小，但是，更新已经响应过了，取到的ns、s都是正确的
				}
			}

			exPulMeasFlag    	=	EXPUL_STA_IED ;                   	//设置标志位，让主程序知道捕获完成
			exPul_TimerOve50ms	=	G_StaV_St.RunTime_50ms + 3;			//设置保护间隔100-150ms
			exPul_TimerOve		=	G_StaV_St.RunTime + exPul_TimerOve50ms / 20 ;
			exPul_TimerOve50ms	=	exPul_TimerOve50ms % 20 ;
		}
		else
		{
			//正常情况不会跑到这里，但是真跑这了，还是把中断清了比较安全
			TIM2->CCER  &= ~(TIM_CCER_CC4E);   	        				//CC4E  关闭捕获
			TIM2->DIER  &= ~(TIM_DIER_CC4IE);            				//CC4IE 关闭中断
			TIM2->SR    &= ~(TIM_SR_CC4IF);              				//清除中断标志位
		}
	}
	else if(TIM2->SR & TIM_SR_UIF)										//本机s脉冲中断
	{
		//本机秒脉冲
		G_StaV_St.NATI_Time++ ;
		TIM2->SR    &= ~(TIM_SR_UIF);            						//清除中断标志位

		if(G_StaV_St.Messa_1PPS == 1)									//置位发送秒脉冲报文
			NaviD_Send_Flag = 1 ;
		else
			NaviD_Send_Flag = 0 ;
	}

	//3.半周期中断比较简单,因为它自己在一边,不会有抢占的风险,所以,不多考虑
	//但是它是调整本机的前沿和GPS秒脉冲前沿对其的重要过程
	//半周期比较中断用于调整TCNT
	if(TIM2->SR & TIM_SR_CC1IF )
	{
		if( TIM2->DIER & TIM_DIER_CC1IE )
		{
			//如果是半周期中断
			Correct_m += Adj_SoftC ;
			TIM2->CNT += Correct_m ;              		//调整
			Correct_m -= Adj_SoftC ;

			if(AcqGPSTime != G_StaV_St.NATI_Time)
			{
				G_StaV_St.NATI_Time = AcqGPSTime ;
			}

			AdjTimeResFlag = 1 ;       					//设置调整标志位，让主程序知道调整完成

			TIM2->CCER	&=	~(TIM_CCER_CC1E);   	    //CC1E  关闭
			TIM2->DIER	&=	~(TIM_DIER_CC1IE);      	//CC1IE 关闭
			TIM2->SR  	&=	~(TIM_SR_CC1IF);     		//清除中断标志位
		}
		else
		{
			//正常情况不会跑到这里，但是真跑这了，还是把中断清了比较安全
			TIM2->CCER	&=	~(TIM_CCER_CC1E);   	    //CC1E  关闭
			TIM2->DIER	&=	~(TIM_DIER_CC1IE);      	//CC1IE 关闭
			TIM2->SR  	&=	~(TIM_SR_CC1IF);     		//清除中断标志位
		}
	}
}
//

//定时器2手动关闭函数，用于测试，快速失配
void TIM2_ONOFF( u8 OnOff )
{
	if(OnOff == 1)
	{
		TIM2->CR1 |= 1 << 0 ;
	}
	else if(OnOff == 0)
	{
		TIM2->CR1 &= ~(1 << 0);
	}
	else
		return;
}
//

////////////////////////////////////////////////
//定时器6,用做运行时钟RunTimer计数
//每50ms中断一次，20次计1s
////////////////////////////////////////////////
void TIM6_50ms_Init(void)
{
	RCC->APB1ENR 	|=  RCC_APB1ENR_TIM6EN ;
	RCC->APB1RSTR	|=  RCC_APB1RSTR_TIM6RST;
	RCC->APB1RSTR	&= ~(RCC_APB1RSTR_TIM6RST);

	TIM6->PSC  = 2000 - 1 ;
	TIM6->ARR  = 1250 - 1 ;
	TIM6->CR1 |= TIM_CR1_ARPE ;

	MY_NVIC_Init(2, 3, TIM6_DAC_IRQn, 2);	//抢占2，子优先级3，组2

	TIM6->EGR  |=  TIM_EGR_UG;            	//UG置1
	TIM6->SR   &= ~(TIM_SR_UIF);          	//清中断标志
	TIM6->DIER |=  TIM_DIER_UIE;          	//开中断

	TIM6->CR1  |=  TIM_CR1_CEN;           	//开计数器
}
//

void TIM6_DAC_IRQHandler(void)
{
	if(TIM6->SR & TIM_SR_UIF  &&  TIM6->DIER & TIM_DIER_UIE ) 			//中断
	{
		TIM6->SR   &= ~(TIM_SR_UIF);                             		//清中断标志

		G_StaV_St.RunTime_50ms++ ;
		G_StaV_St.RunTime_50ms %= 20;

		if(G_StaV_St.RunTime_50ms == 0)
			G_StaV_St.RunTime++;

		LED_ReFresh();								//更新LED显示

		if( G_StaV_St.RunTime_50ms % 10 == 0)
		{
//			ReFresh_ADC();							//刷新ADC采集，这个工程只用了STM内部温度传感器 *此处已注释，使用外部温度传感器
		}
	}

	//两个超时判断
	if(Meas_OVETime != 0)
	{
		Meas_OVETime--;

		if(Meas_OVETime == 0)
		{
			//在这里清除捕获中断
			TIM2->CCER &= ~(TIM_CCER_CC2E);   	  	//CC2E  关闭捕获
			TIM2->DIER &= ~(TIM_DIER_CC2IE);		//CC2IE 关闭中断
			TIM2->SR  &= ~(TIM_SR_CC2IF);        	//清除中断标志位

			AcqTimeResFlag = 2 ;              		//设置超时标志位，让主程序知道测量超时
		}
	}

	if(Adj_OVETime != 0)
	{
		Adj_OVETime--;

		if(Adj_OVETime == 0)
		{
			//在这里关闭半周期中断
			TIM2->CCER	&= ~(TIM_CCER_CC1E);   			//CC1E  关闭
			TIM2->DIER 	&= ~(TIM_DIER_CC1IE);       	//CC1IE 关闭
			TIM2->SR	&= ~(TIM_SR_CC1IF);         	//清除中断标志位

			AdjTimeResFlag  = 2 ;               		//设置超时标志位，让主程序知道测量超时
		}
	}
}
//
//超时检查函数，通用
//把需要到达的时间送入函数
//如果现在的时间超过送入的时间，就返回超时1
//如果现在的时间还没有到达送入的时间，返回没有超时0
u16 CheckTimerOve(u32 Sec, u8 Ms50)
{
	U64 TempNow, TempTim;

	TempNow = G_StaV_St.RunTime * 20 + G_StaV_St.RunTime_50ms;

	TempTim = Sec * 20 + Ms50;

	if( TempNow <= TempTim )
		return 0;
	else
		return 1;
}

//调整量计算程序
//根据周期、第1脉冲宽度，捕获时延，计算需要调整TCNT的数值及方向
//返回有符号数，直接与TCNT相加即可
//具体算法参见规格书中描述,还有笔记本P123/124.
//
s32 AdjCalc(u32 PerT, u32 AcqD, u32 PulW_NAT1P)
{
	u32 CPot, DPot ;                           			//C,D为两个调整阈值点,它们的引入是为了保证,在中期调整的时候
	//不会因为调整幅度过大而马上引起第1脉冲生成,定时器溢出
	//等错误或不良状态

	u32 Tn ;                                  			//无符号捕获值坐标变换,将AcqD这个SYN信号发生在TIM2周期中的位置
	//变换到第1脉冲前沿,发生在SYN周期中的位置.

	s32 Corrm;                                			//有符号修正值,保存了最终的调整值,该值经过阈值点修正

	//CPot = (PerT + 1)/2 - PulW_NAT1P - 500000 ; 		//500000 对应10ms保护距离 计算C点位置
	//DPot = (PerT + 1)/2 - 1        + 500000 ; 		//500000 对应10ms保护距离 计算D点位置

	CPot = (PerT + 1) / 2                 	- 500000 ; 	//500000 对应10ms保护距离 计算C点位置
	DPot = (PerT + 1) / 2 - 1 + PulW_NAT1P  + 500000 ; 	//500000 对应10ms保护距离 计算D点位置

	Tn   = (PerT - AcqD + 1) % (PerT + 1) ;     		//计算Tn,注意若AcqD=0,有可能溢出所以使用%

	if( Tn < CPot )                           			//0<Tn<C,第1脉冲前沿发生在SYN周期中的前半部分且没有达到阈值
	{
		//说明FR信号落后于SYN一小段,FR慢了,需要让FR信号快一点到,TCNT要"+"
		Corrm = Tn ;
	}
	else if( Tn <= (PerT + 1) / 2 )             		//C<=Tn<=(PerT + 1)/2,第1脉冲前沿发生在SYN周期中的前半部分且达到阈值
	{
		//说明FR信号落后于SYN一大段,FR慢了,需要让FR信号快一点到,TCNT要"+",
		Corrm = CPot ;                          		//但是又不能加太多,防止加多了,超过周期1半,向后溢出.
	}
	else if( Tn <= DPot )                     			//(PerT + 1)/2<Tn<=D,第1脉冲前沿发生在SYN周期中的后半部分,且在阈值之前(调整量超过阈值)
	{
		//说明FR信号超前于SYN一大段,FR快了,需要让FR信号慢一点到,TCNT要"-",
		Corrm = -(PerT - DPot + 1) ;           			//但是又不能加太多,防止:1,超过周期1半,向前溢出;2,让TCNT落到第一脉冲期间,引发生成第一脉冲错误.
	}
	else                                      			//D<Tn<=N,第1脉冲前沿发生在SYN周期中的后半部分,且在阈值之后(调整量没有超过阈值)
	{
		//说明FR信号超前于SYN一小段,FR快了,需要让FR信号慢一点到,TCNT要"-",
		Corrm = -(PerT - Tn   + 1) ;            		//因为是一小段,所以直接减去误差量,不会发生错误.
	}

	return Corrm ;                            			//输出修正值,有正有负,经过阈值限幅
}
//

