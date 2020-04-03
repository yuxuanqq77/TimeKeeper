#include "sys.h"

extern u32 Device_SN0;
extern u32 Device_SN1;
extern u32 Device_SN2;

//////////////////////////////////////
//芯片串号验证锁死函数
void Get_SerialNum(void)
{
	u32 Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(vu32*)(0x1FFF7A10);      //12 Bytes Serial Number
	Device_Serial1 = *(vu32*)(0x1FFF7A14);
	Device_Serial2 = *(vu32*)(0x1FFF7A28);

	if (Device_Serial0 != 0)
	{
		Device_SN0 = Device_Serial0;
		Device_SN1 = Device_Serial1;
		Device_SN2 = Device_Serial2;
	}

	if( Device_SN0 == 0x00280023 &&
			Device_SN1 == 0x30365110 &&
			Device_SN2 == 0x05E3F000 )
		return;
	else
		while(1) {};
}

//设置向量表偏移地址
//NVIC_VectTab:基址
//Offset:偏移量
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{
	SCB->VTOR = NVIC_VectTab | (Offset & (u32)0xFFFFFE00); //设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留。
}
//设置NVIC分组
//NVIC_Group:NVIC分组 0~4 总共5组
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
	u32 temp, temp1;
	temp1 = (~NVIC_Group) & 0x07; //取后三位
	temp1 <<= 8;
	temp = SCB->AIRCR; //读取先前的设置
	temp &= 0X0000F8FF; //清空先前分组
	temp |= 0X05FA0000; //写入钥匙
	temp |= temp1;
	SCB->AIRCR = temp; //设置分组
}
//设置NVIC
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group)
{
	u32 temp;
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
	temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
	temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
	temp &= 0xf;								//取低四位
	NVIC->ISER[NVIC_Channel / 32] |= 1 << NVIC_Channel % 32; //使能中断位(要清除的话,设置ICER对应位为1即可)
	NVIC->IP[NVIC_Channel] |= temp << 4;				//设置响应优先级和抢断优先级
}
//外部中断配置函数
//只针对GPIOA~I;不包括PVD,RTC,USB_OTG,USB_HS,以太网唤醒等
//参数:
//GPIOx:0~8,代表GPIOA~I
//BITx:需要使能的位;
//TRIM:触发模式,1,下升沿;2,上降沿;3，任意电平触发
//该函数一次只能配置1个IO口,多个IO口,需多次调用
//该函数会自动开启对应中断,以及屏蔽线
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM)
{
	u8 EXTOFFSET = (BITx % 4) * 4;
	RCC->APB2ENR |= 1 << 14;  						//使能SYSCFG时钟
	SYSCFG->EXTICR[BITx / 4] &= ~(0x000F << EXTOFFSET); //清除原来设置！！！
	SYSCFG->EXTICR[BITx / 4] |= GPIOx << EXTOFFSET;	//EXTI.BITx映射到GPIOx.BITx
	//自动设置
	EXTI->IMR |= 1 << BITx;					//开启line BITx上的中断(如果要禁止中断，则反操作即可)

	if(TRIM & 0x01)EXTI->FTSR |= 1 << BITx;	//line BITx上事件下降沿触发

	if(TRIM & 0x02)EXTI->RTSR |= 1 << BITx;	//line BITx上事件上升降沿触发
}
//GPIO复用设置
//GPIOx:GPIOA~GPIOI.
//BITx:0~15,代表IO引脚编号.
//AFx:0~15,代表AF0~AF15.
//AF0~15设置情况(这里仅是列出常用的,详细的请见407数据手册,56页Table 7):
//AF0:MCO/SWD/SWCLK/RTC   AF1:TIM1/TIM2;            AF2:TIM3~5;               AF3:TIM8~11
//AF4:I2C1~I2C3;          AF5:SPI1/SPI2;            AF6:SPI3;                 AF7:USART1~3;
//AF8:USART4~6;           AF9;CAN1/CAN2/TIM12~14    AF10:USB_OTG/USB_HS       AF11:ETH
//AF12:FSMC/SDIO/OTG/HS   AF13:DCIM                 AF14:                     AF15:EVENTOUT
void GPIO_AF_Set(GPIO_TypeDef* GPIOx, u8 BITx, u8 AFx)
{
	GPIOx->AFR[BITx >> 3] &= ~(0X0F << ((BITx & 0X07) * 4));
	GPIOx->AFR[BITx >> 3] |= (u32)AFx << ((BITx & 0X07) * 4);
}
//GPIO通用设置
//GPIOx:GPIOA~GPIOI.
//BITx:0X0000~0XFFFF,位设置,每个位代表一个IO,第0位代表Px0,第1位代表Px1,依次类推.比如0X0101,代表同时设置Px0和Px8.
//MODE:0~3;模式选择,0,输入(系统复位默认状态);1,普通输出;2,复用功能;3,模拟输入.
//OTYPE:0/1;输出类型选择,0,推挽输出;1,开漏输出.
//OSPEED:0~3;输出速度设置,0,2Mhz;1,25Mhz;2,50Mhz;3,100Mh.
//PUPD:0~3:上下拉设置,0,不带上下拉;1,上拉;2,下拉;3,保留.
//注意:在输入模式(普通输入/模拟输入)下,OTYPE和OSPEED参数无效!!
void GPIO_Set(GPIO_TypeDef* GPIOx, u32 BITx, u32 MODE, u32 OTYPE, u32 OSPEED, u32 PUPD)
{
	u32 pinpos = 0, pos = 0, curpin = 0;

	for(pinpos = 0; pinpos < 16; pinpos++)
	{
		pos = 1 << pinpos;	//一个个位检查
		curpin = BITx & pos; //检查引脚是否要设置

		if(curpin == pos)	//需要设置
		{
			GPIOx->MODER &= ~(3 << (pinpos * 2));	//先清除原来的设置
			GPIOx->MODER |= MODE << (pinpos * 2);	//设置新的模式

			if((MODE == 0X01) || (MODE == 0X02))	//如果是输出模式/复用功能模式
			{
				GPIOx->OSPEEDR &= ~(3 << (pinpos * 2));	//清除原来的设置
				GPIOx->OSPEEDR |= (OSPEED << (pinpos * 2)); //设置新的速度值
				GPIOx->OTYPER &= ~(1 << pinpos) ;		//清除原来的设置
				GPIOx->OTYPER |= OTYPE << pinpos;		//设置新的输出模式
			}

			GPIOx->PUPDR &= ~(3 << (pinpos * 2));	//先清除原来的设置
			GPIOx->PUPDR |= PUPD << (pinpos * 2);	//设置新的上下拉
		}
	}
}

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI
__asm void WFI_SET(void)
{
	WFI;
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr)
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
//进入待机模式
void Sys_Standby(void)
{
	SCB->SCR |= 1 << 2;		//使能SLEEPDEEP位 (SYS->CTRL)
	RCC->APB1ENR |= 1 << 28; //使能电源时钟
	PWR->CSR |= 1 << 8; //设置WKUP用于唤醒
	PWR->CR |= 1 << 2;  //清除Wake-up 标志
	PWR->CR |= 1 << 1;  //PDDS置位
	WFI_SET();			//执行WFI指令,进入待机模式
}
//系统软复位
void Sys_Soft_Reset(void)
{
	SCB->AIRCR = 0X05FA0000 | (u32)0x04;
}
//时钟设置函数
//Fvco=Fs*(plln/pllm);
//Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCO频率
//Fsys:系统时钟频率
//Fusb:USB,SDIO,RNG等的时钟频率
//Fs:PLL输入时钟频率,可以是HSI,HSE等.
//plln:主PLL倍频系数(PLL倍频),取值范围:64~432.
//pllm:主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
//pllp:系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
//pllq:USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.

//外部晶振为8M的时候,推荐值:plln=336,pllm=8,pllp=2,pllq=7.
//得到:Fvco=8*(336/8)=336Mhz
//     Fsys=336/2=168Mhz
//     Fusb=336/7=48Mhz
//返回值:0,成功;1,失败。
u8 Sys_Clock_Set(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	u16 retry = 0;
	u8 status = 0;
	RCC->CR |= 1 << 16;				//HSE 开启

	while(((RCC->CR & (1 << 17)) == 0) && (retry < 0X1FFF))
		retry++;//等待HSE RDY

	if(retry == 0X1FFF)
		status = 1;	//HSE无法就绪
	else
	{
		//为了省电，此处关闭
		//RCC->APB1ENR|=1<<28;		//电源接口时钟使能
		//PWR->CR|=3<<14; 		  	//高性能模式,时钟可到168Mhz

		RCC->CFGR |= (0 << 4) | (4 << 10) | (4 << 13);	//HCLK 不分频;APB1 2分频;APB2 2分频.
		RCC->CR &= ~(1 << 24);	              	//关闭主PLL

		RCC->PLLCFGR = pllm | (plln << 6) | (((pllp >> 1) - 1) << 16) | (pllq << 24) | (1 << 22); //配置主PLL,PLL时钟源来自HSE

		RCC->CR |= 1 << 24;			//打开主PLL

		while((RCC->CR & (1 << 25)) == 0)		//等待PLL准备好
		{};

		FLASH->ACR |= 1 << 8;				//指令预取使能.

		FLASH->ACR |= 1 << 9;				//指令cache使能.

		FLASH->ACR |= 1 << 10;				//数据cache使能.

		FLASH->ACR |= 5 << 0;				//5个CPU等待周期.

		RCC->CFGR &= ~(3 << 0);				//清零

		RCC->CFGR |= 2 << 0;				//选择主PLL作为系统时钟

		while((RCC->CFGR & (3 << 2)) != (2 << 2))	//等待主PLL作为系统时钟成功.
		{};
	}

	return status;
}

u8 Sys_Clock_Set_HSI(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	u16 retry = 0;
	u8 status = 0;

	while(((RCC->CR & (1 << 1)) == 0) && (retry < 0X1FFF))
		retry++;//等待HSI RDY

	if(retry == 0X1FFF)
		status = 1;	//HSI无法就绪
	else
	{
		//RCC->APB1ENR|=1<<28;	    //电源接口时钟使能
		//PWR->CR|=3<<14; 		    //高性能模式,时钟可到168Mhz

		RCC->CFGR   |=  (0 << 4) | (4 << 10) | (4 << 13); //HCLK 不分频;APB1 2分频;APB2 2分频.

		RCC->CR     &= ~(1 << 24);	              //关闭主PLL

		RCC->PLLCFGR =   pllm | (plln << 6) | (((pllp >> 1) - 1) << 16) | (pllq << 24) | (0 << 22); //配置主PLL,PLL时钟源来自HSI

		RCC->CR |= 1 << 24;			//打开主PLL

		while((RCC->CR & (1 << 25)) == 0); //等待PLL准备好

		FLASH->ACR |= 1 << 8;		//指令预取使能.
		FLASH->ACR |= 1 << 9;		//指令cache使能.
		FLASH->ACR |= 1 << 10;		//数据cache使能.
		FLASH->ACR |= 5 << 0;		//5个CPU等待周期.

		RCC->CFGR   &=  ~(3 << 0);  //清零
		RCC->CFGR   |=    2 << 0;	  //选择主PLL作为系统时钟

		while((RCC->CFGR & (3 << 2)) != (2 << 2)); //等待主PLL作为系统时钟成功.

	}

	return status;
}

//系统时钟初始化函数
//plln:主PLL倍频系数(PLL倍频),取值范围:64~432.
//pllm:主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
//pllp:系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
//pllq:USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.
void Stm32_Clock_Init(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	RCC->CR |= 0x00000001;		//设置HISON,开启内部高速RC振荡
	RCC->CFGR = 0x00000000;		//CFGR清零
	RCC->CR &= 0xFEF6FFFF;		//HSEON,CSSON,PLLON清零

	RCC->PLLCFGR = 0x24003010;	//PLLCFGR恢复复位值
	//RCC->CR&=~(1<<18);	    //HSEBYP清零,外部晶振不旁路
	RCC->CR |= (1 << 18);		//HSEBYP置1,有源晶体提供方波
	RCC->CIR = 0x00000000;		//禁止RCC时钟中断

	Sys_Clock_Set(plln, pllm, pllp, pllq); //设置时钟

	//配置向量表
	#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(1 << 29, 0x0);
	#else
	MY_NVIC_SetVectorTable(0, 0x0);
	#endif
}




void Stm32_Clock_Init_HSI(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	RCC->CR     |=  0x00000001;		//设置HISON,开启内部高速RC振荡
	RCC->CFGR    =  0x00000000;		//CFGR清零
	RCC->CR     &=  0xFEF6FFFF;		//HSEON,CSSON,PLLON清零

	RCC->PLLCFGR =  0x24003010;	    //PLLCFGR恢复复位值

	RCC->CIR     =  0x00000000;		//禁止RCC时钟中断

	Sys_Clock_Set_HSI(plln, pllm, pllp, pllq); //设置时钟

	//配置向量表
	#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(1 << 29, 0x0);
	#else
	MY_NVIC_SetVectorTable(0, 0x0);
	#endif
}

//月份数据表
u8 const table_week[12] = {0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5}; //月修正数据表
//平年的月份日期表
const u8 mon_table[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

u8 Is_Leap_Year(u16 year)
{
	if(year % 4 == 0) //必须能被4整除
	{
		if(year % 100 == 0)
		{
			if(year % 400 == 0)return 1; //如果以00结尾,还要能被400整除
			else return 0;
		}
		else return 1;
	}
	else return 0;
}
//获得现在是星期几
//功能描述:输入公历日期得到星期(只允许1901-2099年)
//year,month,day：公历年月日
//返回值：星期号
u8 RTC_Get_Week(u16 year, u8 month, u8 day)
{
	u16 temp2;
	u8 yearH, yearL;

	yearH = year / 100;
	yearL = year % 100;

	// 如果为21世纪,年份数加100
	if (yearH > 19)yearL += 100;

	// 所过闰年数只算1900年之后的
	temp2 = yearL + yearL / 4;
	temp2 = temp2 % 7;
	temp2 = temp2 + day + table_week[month - 1];

	if (yearL % 4 == 0 && month < 3)temp2--;

	return(temp2 % 7);
}

//输入UNIX时间,输出RTC时间
void UnixT2Rtc(RTC_TIME_Str * ReadTime)
{
	u32 temp = 0;
	u16 temp1 = 0;
	u32 timecount;

	timecount = ReadTime->Unix_Time;

	temp 	= timecount / 86400; 	//得到天数(秒钟数对应的)
	temp1 	= 1970;	          		//从1970年开始

	while(temp >= 365)
	{
		if(Is_Leap_Year(temp1))   	//是闰年
		{
			if(temp >= 366)
				temp -= 366;      	//闰年的秒钟数
			else
				break;
		}
		else temp -= 365;	        //平年

		temp1++;
	}

	ReadTime->QYear = temp1;    	//得到年份

	temp1 = 0;

	while(temp >= 28)           	//超过了一个月
	{
		if(Is_Leap_Year(ReadTime->QYear) && temp1 == 1) //当年是不是闰年/2月份
		{
			if(temp >= 29)temp -= 29;                 	//闰年的秒钟数
			else break;
		}
		else
		{
			if(temp >= mon_table[temp1])temp -= mon_table[temp1]; //平年
			else break;
		}

		temp1++;
	}

	ReadTime->Mon = temp1 + 1;	  			//得到月份
	ReadTime->Dat = temp + 1;  	 			//得到日期

	temp = timecount % 86400;     		 	//得到秒钟数

	ReadTime->Hor = temp / 3600;     	 	//小时
	ReadTime->Min = (temp % 3600) / 60; 	//分钟
	ReadTime->Sec = (temp % 3600) % 60; 	//秒钟
	ReadTime->Day = RTC_Get_Week(ReadTime->QYear, ReadTime->Mon, ReadTime->Dat); //获取星期
	ReadTime->Yea = ReadTime->QYear % 100;
}

//输入RTC时间,输出UNIX时间
u32 Rtc2UnixT(RTC_TIME_Str * ReadTime)
{
	u16 t;
	u32 seccount = 0;

	if(ReadTime->QYear < 1970 || ReadTime->QYear > 2099)
		return 0;

	for(t = 1970; t < ReadTime->QYear; t++)			//把所有年份的秒钟相加
	{
		if(Is_Leap_Year(t))
			seccount += 31622400;         			//闰年的秒钟数
		else
			seccount += 31536000;			 		//平年的秒钟数
	}

	for(t = 0; t < (ReadTime->Mon - 1); t++)		//把前面月份的秒钟数相加
	{
		seccount += (u32)mon_table[t] * 86400; 		//月份秒钟数相加

		if(Is_Leap_Year(ReadTime->QYear) && t == 1)
			seccount += 86400;              		//闰年2月份增加一天的秒钟数
	}

	seccount += (u32)(ReadTime->Dat - 1) * 86400; 	//把前面日期的秒钟数相加
	seccount += (u32)(ReadTime->Hor) * 3600; 		//小时秒钟数
	seccount += (u32)(ReadTime->Min) * 60;			//分钟秒钟数
	seccount += (ReadTime->Sec);            		//最后的秒钟加上去

	ReadTime->Unix_Time = seccount ;
	return seccount;
}
//

u8  LED_Per[2] = {0, 0};
u8  LED_Sat[2] = {0, 0};

//LED IO初始化
void LED_Init(void)
{
	RCC->AHB1ENR |= 1 << 0; //使能PORTA时钟
	GPIO_Set(GPIOA, PIN12 | PIN11, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); //

	LED1 = 0; //LED1打开
	LED2 = 0; //LED2打开

	LED1 = 1; //LED1关闭
	LED2 = 1; //LED2关闭
}

//在50ms中断中刷新LED状态
void LED_ReFresh(void)
{
	if( LED_Per[0] == 0)
		LED1 = 1 ;
	else if( LED_Per[0] == 1)
		LED1 = 0 ;
	else
	{
		if(LED_Sat[0] == 0)
			LED1 = 0 ;
		else
			LED1 = 1 ;

		LED_Sat[0]++;
		LED_Sat[0] %= LED_Per[0] ;
	}

	if( LED_Per[1] == 0)
		LED2 = 1 ;
	else if( LED_Per[1] == 1)
		LED2 = 0 ;
	else
	{
		if(LED_Sat[1] == 0)
			LED2 = 0 ;
		else
			LED2 = 1 ;

		LED_Sat[1]++;
		LED_Sat[1] %= LED_Per[1] ;
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* Function:    	void getCRCtable2(const unsigned int crcmask, const unsigned int crcn, unsigned int ptable[])
* Description:    	生成CRC校验和计算过程中需要的数据表
* Input par:
unsigned int crcmask:    	输入CRC计算多项式掩码
unsigned int crcn:        	输入CRC位数,单位:比特
unsigned int ptable:     	存储数据表指针
* Return:        	void
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
__align(4)	u32		crcTalbe[256];

void getCRCtable2(u16 crcmask, u16 crcn, u32 ptable[])
{
	unsigned int chtmp, crc, crcHB;
	int i, j;

	chtmp = 0;

	for (j = 0; j < 0x100; j++)
	{
		crc = chtmp++;

		for (i = 0; i < 8; i++)
		{
			crcHB = (crc & 0x01) * crcmask;
			crc >>= 1;
			crc ^= crcHB;
		}

		*ptable++ = crc & ((1 << crcn) - 1);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*2^(n-1-3)-1 byte
* Function:    	unsigned char getCRC16(const unsigned char input[], unsigned int inputlen, const unsigned int ptable[])
* Description:    	CRC 16校验和计算.最大输入数据长度4K-1 byte(2^(16-1)-1) bit).目前都是以整字节计算以方便实现.
* Input par:
const unsigned char input:	输入数据缓存
unsigned int inputlen:    	输入数据长度,单位:字节
const unsigned int ptable: 	校验和计算用数据表,由函数getCRCtable2()生成
* Return:        	unsigned short :        	CRC 16 检验和
* Note:
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
u16 getCRC16(u8 *pinput, u16 inputlen, u32 ptable[])
{
	unsigned int i;
	unsigned char chtmp;

	unsigned int low8;
	unsigned int high8;				//must be unsigned
	unsigned int crc = 0xffff;		//not 0

	for (i = 0; i < inputlen; i++)
	{
		chtmp = *pinput++;
		high8 = (crc & 0xff) ^ chtmp;
		low8 = crc >> 8;
		crc = low8 ^ (ptable[high8]);

	}

	return (unsigned short)(crc & (0xffff));
}

