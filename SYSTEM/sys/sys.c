#include "sys.h"

extern u32 Device_SN0;
extern u32 Device_SN1;
extern u32 Device_SN2;

//////////////////////////////////////
//оƬ������֤��������
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

//����������ƫ�Ƶ�ַ
//NVIC_VectTab:��ַ
//Offset:ƫ����
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{
	SCB->VTOR = NVIC_VectTab | (Offset & (u32)0xFFFFFE00); //����NVIC��������ƫ�ƼĴ���,VTOR��9λ����,��[8:0]������
}
//����NVIC����
//NVIC_Group:NVIC���� 0~4 �ܹ�5��
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
	u32 temp, temp1;
	temp1 = (~NVIC_Group) & 0x07; //ȡ����λ
	temp1 <<= 8;
	temp = SCB->AIRCR; //��ȡ��ǰ������
	temp &= 0X0000F8FF; //�����ǰ����
	temp |= 0X05FA0000; //д��Կ��
	temp |= temp1;
	SCB->AIRCR = temp; //���÷���
}
//����NVIC
//NVIC_PreemptionPriority:��ռ���ȼ�
//NVIC_SubPriority       :��Ӧ���ȼ�
//NVIC_Channel           :�жϱ��
//NVIC_Group             :�жϷ��� 0~4
//ע�����ȼ����ܳ����趨����ķ�Χ!����������벻���Ĵ���
//�黮��:
//��0:0λ��ռ���ȼ�,4λ��Ӧ���ȼ�
//��1:1λ��ռ���ȼ�,3λ��Ӧ���ȼ�
//��2:2λ��ռ���ȼ�,2λ��Ӧ���ȼ�
//��3:3λ��ռ���ȼ�,1λ��Ӧ���ȼ�
//��4:4λ��ռ���ȼ�,0λ��Ӧ���ȼ�
//NVIC_SubPriority��NVIC_PreemptionPriority��ԭ����,��ֵԽС,Խ����
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group)
{
	u32 temp;
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//���÷���
	temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
	temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
	temp &= 0xf;								//ȡ����λ
	NVIC->ISER[NVIC_Channel / 32] |= 1 << NVIC_Channel % 32; //ʹ���ж�λ(Ҫ����Ļ�,����ICER��ӦλΪ1����)
	NVIC->IP[NVIC_Channel] |= temp << 4;				//������Ӧ���ȼ����������ȼ�
}
//�ⲿ�ж����ú���
//ֻ���GPIOA~I;������PVD,RTC,USB_OTG,USB_HS,��̫�����ѵ�
//����:
//GPIOx:0~8,����GPIOA~I
//BITx:��Ҫʹ�ܵ�λ;
//TRIM:����ģʽ,1,������;2,�Ͻ���;3�������ƽ����
//�ú���һ��ֻ������1��IO��,���IO��,���ε���
//�ú������Զ�������Ӧ�ж�,�Լ�������
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM)
{
	u8 EXTOFFSET = (BITx % 4) * 4;
	RCC->APB2ENR |= 1 << 14;  						//ʹ��SYSCFGʱ��
	SYSCFG->EXTICR[BITx / 4] &= ~(0x000F << EXTOFFSET); //���ԭ�����ã�����
	SYSCFG->EXTICR[BITx / 4] |= GPIOx << EXTOFFSET;	//EXTI.BITxӳ�䵽GPIOx.BITx
	//�Զ�����
	EXTI->IMR |= 1 << BITx;					//����line BITx�ϵ��ж�(���Ҫ��ֹ�жϣ��򷴲�������)

	if(TRIM & 0x01)EXTI->FTSR |= 1 << BITx;	//line BITx���¼��½��ش���

	if(TRIM & 0x02)EXTI->RTSR |= 1 << BITx;	//line BITx���¼��������ش���
}
//GPIO��������
//GPIOx:GPIOA~GPIOI.
//BITx:0~15,����IO���ű��.
//AFx:0~15,����AF0~AF15.
//AF0~15�������(��������г����õ�,��ϸ�����407�����ֲ�,56ҳTable 7):
//AF0:MCO/SWD/SWCLK/RTC   AF1:TIM1/TIM2;            AF2:TIM3~5;               AF3:TIM8~11
//AF4:I2C1~I2C3;          AF5:SPI1/SPI2;            AF6:SPI3;                 AF7:USART1~3;
//AF8:USART4~6;           AF9;CAN1/CAN2/TIM12~14    AF10:USB_OTG/USB_HS       AF11:ETH
//AF12:FSMC/SDIO/OTG/HS   AF13:DCIM                 AF14:                     AF15:EVENTOUT
void GPIO_AF_Set(GPIO_TypeDef* GPIOx, u8 BITx, u8 AFx)
{
	GPIOx->AFR[BITx >> 3] &= ~(0X0F << ((BITx & 0X07) * 4));
	GPIOx->AFR[BITx >> 3] |= (u32)AFx << ((BITx & 0X07) * 4);
}
//GPIOͨ������
//GPIOx:GPIOA~GPIOI.
//BITx:0X0000~0XFFFF,λ����,ÿ��λ����һ��IO,��0λ����Px0,��1λ����Px1,��������.����0X0101,����ͬʱ����Px0��Px8.
//MODE:0~3;ģʽѡ��,0,����(ϵͳ��λĬ��״̬);1,��ͨ���;2,���ù���;3,ģ������.
//OTYPE:0/1;�������ѡ��,0,�������;1,��©���.
//OSPEED:0~3;����ٶ�����,0,2Mhz;1,25Mhz;2,50Mhz;3,100Mh.
//PUPD:0~3:����������,0,����������;1,����;2,����;3,����.
//ע��:������ģʽ(��ͨ����/ģ������)��,OTYPE��OSPEED������Ч!!
void GPIO_Set(GPIO_TypeDef* GPIOx, u32 BITx, u32 MODE, u32 OTYPE, u32 OSPEED, u32 PUPD)
{
	u32 pinpos = 0, pos = 0, curpin = 0;

	for(pinpos = 0; pinpos < 16; pinpos++)
	{
		pos = 1 << pinpos;	//һ����λ���
		curpin = BITx & pos; //��������Ƿ�Ҫ����

		if(curpin == pos)	//��Ҫ����
		{
			GPIOx->MODER &= ~(3 << (pinpos * 2));	//�����ԭ��������
			GPIOx->MODER |= MODE << (pinpos * 2);	//�����µ�ģʽ

			if((MODE == 0X01) || (MODE == 0X02))	//��������ģʽ/���ù���ģʽ
			{
				GPIOx->OSPEEDR &= ~(3 << (pinpos * 2));	//���ԭ��������
				GPIOx->OSPEEDR |= (OSPEED << (pinpos * 2)); //�����µ��ٶ�ֵ
				GPIOx->OTYPER &= ~(1 << pinpos) ;		//���ԭ��������
				GPIOx->OTYPER |= OTYPE << pinpos;		//�����µ����ģʽ
			}

			GPIOx->PUPDR &= ~(3 << (pinpos * 2));	//�����ԭ��������
			GPIOx->PUPDR |= PUPD << (pinpos * 2);	//�����µ�������
		}
	}
}

//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI
__asm void WFI_SET(void)
{
	WFI;
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr)
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
//�������ģʽ
void Sys_Standby(void)
{
	SCB->SCR |= 1 << 2;		//ʹ��SLEEPDEEPλ (SYS->CTRL)
	RCC->APB1ENR |= 1 << 28; //ʹ�ܵ�Դʱ��
	PWR->CSR |= 1 << 8; //����WKUP���ڻ���
	PWR->CR |= 1 << 2;  //���Wake-up ��־
	PWR->CR |= 1 << 1;  //PDDS��λ
	WFI_SET();			//ִ��WFIָ��,�������ģʽ
}
//ϵͳ��λ
void Sys_Soft_Reset(void)
{
	SCB->AIRCR = 0X05FA0000 | (u32)0x04;
}
//ʱ�����ú���
//Fvco=Fs*(plln/pllm);
//Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCOƵ��
//Fsys:ϵͳʱ��Ƶ��
//Fusb:USB,SDIO,RNG�ȵ�ʱ��Ƶ��
//Fs:PLL����ʱ��Ƶ��,������HSI,HSE��.
//plln:��PLL��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:64~432.
//pllm:��PLL����ƵPLL��Ƶϵ��(PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
//pllp:ϵͳʱ�ӵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2,4,6,8.(������4��ֵ!)
//pllq:USB/SDIO/������������ȵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2~15.

//�ⲿ����Ϊ8M��ʱ��,�Ƽ�ֵ:plln=336,pllm=8,pllp=2,pllq=7.
//�õ�:Fvco=8*(336/8)=336Mhz
//     Fsys=336/2=168Mhz
//     Fusb=336/7=48Mhz
//����ֵ:0,�ɹ�;1,ʧ�ܡ�
u8 Sys_Clock_Set(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	u16 retry = 0;
	u8 status = 0;
	RCC->CR |= 1 << 16;				//HSE ����

	while(((RCC->CR & (1 << 17)) == 0) && (retry < 0X1FFF))
		retry++;//�ȴ�HSE RDY

	if(retry == 0X1FFF)
		status = 1;	//HSE�޷�����
	else
	{
		//Ϊ��ʡ�磬�˴��ر�
		//RCC->APB1ENR|=1<<28;		//��Դ�ӿ�ʱ��ʹ��
		//PWR->CR|=3<<14; 		  	//������ģʽ,ʱ�ӿɵ�168Mhz

		RCC->CFGR |= (0 << 4) | (4 << 10) | (4 << 13);	//HCLK ����Ƶ;APB1 2��Ƶ;APB2 2��Ƶ.
		RCC->CR &= ~(1 << 24);	              	//�ر���PLL

		RCC->PLLCFGR = pllm | (plln << 6) | (((pllp >> 1) - 1) << 16) | (pllq << 24) | (1 << 22); //������PLL,PLLʱ��Դ����HSE

		RCC->CR |= 1 << 24;			//����PLL

		while((RCC->CR & (1 << 25)) == 0)		//�ȴ�PLL׼����
		{};

		FLASH->ACR |= 1 << 8;				//ָ��Ԥȡʹ��.

		FLASH->ACR |= 1 << 9;				//ָ��cacheʹ��.

		FLASH->ACR |= 1 << 10;				//����cacheʹ��.

		FLASH->ACR |= 5 << 0;				//5��CPU�ȴ�����.

		RCC->CFGR &= ~(3 << 0);				//����

		RCC->CFGR |= 2 << 0;				//ѡ����PLL��Ϊϵͳʱ��

		while((RCC->CFGR & (3 << 2)) != (2 << 2))	//�ȴ���PLL��Ϊϵͳʱ�ӳɹ�.
		{};
	}

	return status;
}

u8 Sys_Clock_Set_HSI(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	u16 retry = 0;
	u8 status = 0;

	while(((RCC->CR & (1 << 1)) == 0) && (retry < 0X1FFF))
		retry++;//�ȴ�HSI RDY

	if(retry == 0X1FFF)
		status = 1;	//HSI�޷�����
	else
	{
		//RCC->APB1ENR|=1<<28;	    //��Դ�ӿ�ʱ��ʹ��
		//PWR->CR|=3<<14; 		    //������ģʽ,ʱ�ӿɵ�168Mhz

		RCC->CFGR   |=  (0 << 4) | (4 << 10) | (4 << 13); //HCLK ����Ƶ;APB1 2��Ƶ;APB2 2��Ƶ.

		RCC->CR     &= ~(1 << 24);	              //�ر���PLL

		RCC->PLLCFGR =   pllm | (plln << 6) | (((pllp >> 1) - 1) << 16) | (pllq << 24) | (0 << 22); //������PLL,PLLʱ��Դ����HSI

		RCC->CR |= 1 << 24;			//����PLL

		while((RCC->CR & (1 << 25)) == 0); //�ȴ�PLL׼����

		FLASH->ACR |= 1 << 8;		//ָ��Ԥȡʹ��.
		FLASH->ACR |= 1 << 9;		//ָ��cacheʹ��.
		FLASH->ACR |= 1 << 10;		//����cacheʹ��.
		FLASH->ACR |= 5 << 0;		//5��CPU�ȴ�����.

		RCC->CFGR   &=  ~(3 << 0);  //����
		RCC->CFGR   |=    2 << 0;	  //ѡ����PLL��Ϊϵͳʱ��

		while((RCC->CFGR & (3 << 2)) != (2 << 2)); //�ȴ���PLL��Ϊϵͳʱ�ӳɹ�.

	}

	return status;
}

//ϵͳʱ�ӳ�ʼ������
//plln:��PLL��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:64~432.
//pllm:��PLL����ƵPLL��Ƶϵ��(PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
//pllp:ϵͳʱ�ӵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2,4,6,8.(������4��ֵ!)
//pllq:USB/SDIO/������������ȵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2~15.
void Stm32_Clock_Init(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	RCC->CR |= 0x00000001;		//����HISON,�����ڲ�����RC��
	RCC->CFGR = 0x00000000;		//CFGR����
	RCC->CR &= 0xFEF6FFFF;		//HSEON,CSSON,PLLON����

	RCC->PLLCFGR = 0x24003010;	//PLLCFGR�ָ���λֵ
	//RCC->CR&=~(1<<18);	    //HSEBYP����,�ⲿ������·
	RCC->CR |= (1 << 18);		//HSEBYP��1,��Դ�����ṩ����
	RCC->CIR = 0x00000000;		//��ֹRCCʱ���ж�

	Sys_Clock_Set(plln, pllm, pllp, pllq); //����ʱ��

	//����������
	#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(1 << 29, 0x0);
	#else
	MY_NVIC_SetVectorTable(0, 0x0);
	#endif
}




void Stm32_Clock_Init_HSI(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
	RCC->CR     |=  0x00000001;		//����HISON,�����ڲ�����RC��
	RCC->CFGR    =  0x00000000;		//CFGR����
	RCC->CR     &=  0xFEF6FFFF;		//HSEON,CSSON,PLLON����

	RCC->PLLCFGR =  0x24003010;	    //PLLCFGR�ָ���λֵ

	RCC->CIR     =  0x00000000;		//��ֹRCCʱ���ж�

	Sys_Clock_Set_HSI(plln, pllm, pllp, pllq); //����ʱ��

	//����������
	#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(1 << 29, 0x0);
	#else
	MY_NVIC_SetVectorTable(0, 0x0);
	#endif
}

//�·����ݱ�
u8 const table_week[12] = {0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5}; //���������ݱ�
//ƽ����·����ڱ�
const u8 mon_table[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

u8 Is_Leap_Year(u16 year)
{
	if(year % 4 == 0) //�����ܱ�4����
	{
		if(year % 100 == 0)
		{
			if(year % 400 == 0)return 1; //�����00��β,��Ҫ�ܱ�400����
			else return 0;
		}
		else return 1;
	}
	else return 0;
}
//������������ڼ�
//��������:���빫�����ڵõ�����(ֻ����1901-2099��)
//year,month,day������������
//����ֵ�����ں�
u8 RTC_Get_Week(u16 year, u8 month, u8 day)
{
	u16 temp2;
	u8 yearH, yearL;

	yearH = year / 100;
	yearL = year % 100;

	// ���Ϊ21����,�������100
	if (yearH > 19)yearL += 100;

	// ����������ֻ��1900��֮���
	temp2 = yearL + yearL / 4;
	temp2 = temp2 % 7;
	temp2 = temp2 + day + table_week[month - 1];

	if (yearL % 4 == 0 && month < 3)temp2--;

	return(temp2 % 7);
}

//����UNIXʱ��,���RTCʱ��
void UnixT2Rtc(RTC_TIME_Str * ReadTime)
{
	u32 temp = 0;
	u16 temp1 = 0;
	u32 timecount;

	timecount = ReadTime->Unix_Time;

	temp 	= timecount / 86400; 	//�õ�����(��������Ӧ��)
	temp1 	= 1970;	          		//��1970�꿪ʼ

	while(temp >= 365)
	{
		if(Is_Leap_Year(temp1))   	//������
		{
			if(temp >= 366)
				temp -= 366;      	//�����������
			else
				break;
		}
		else temp -= 365;	        //ƽ��

		temp1++;
	}

	ReadTime->QYear = temp1;    	//�õ����

	temp1 = 0;

	while(temp >= 28)           	//������һ����
	{
		if(Is_Leap_Year(ReadTime->QYear) && temp1 == 1) //�����ǲ�������/2�·�
		{
			if(temp >= 29)temp -= 29;                 	//�����������
			else break;
		}
		else
		{
			if(temp >= mon_table[temp1])temp -= mon_table[temp1]; //ƽ��
			else break;
		}

		temp1++;
	}

	ReadTime->Mon = temp1 + 1;	  			//�õ��·�
	ReadTime->Dat = temp + 1;  	 			//�õ�����

	temp = timecount % 86400;     		 	//�õ�������

	ReadTime->Hor = temp / 3600;     	 	//Сʱ
	ReadTime->Min = (temp % 3600) / 60; 	//����
	ReadTime->Sec = (temp % 3600) % 60; 	//����
	ReadTime->Day = RTC_Get_Week(ReadTime->QYear, ReadTime->Mon, ReadTime->Dat); //��ȡ����
	ReadTime->Yea = ReadTime->QYear % 100;
}

//����RTCʱ��,���UNIXʱ��
u32 Rtc2UnixT(RTC_TIME_Str * ReadTime)
{
	u16 t;
	u32 seccount = 0;

	if(ReadTime->QYear < 1970 || ReadTime->QYear > 2099)
		return 0;

	for(t = 1970; t < ReadTime->QYear; t++)			//��������ݵ��������
	{
		if(Is_Leap_Year(t))
			seccount += 31622400;         			//�����������
		else
			seccount += 31536000;			 		//ƽ���������
	}

	for(t = 0; t < (ReadTime->Mon - 1); t++)		//��ǰ���·ݵ����������
	{
		seccount += (u32)mon_table[t] * 86400; 		//�·����������

		if(Is_Leap_Year(ReadTime->QYear) && t == 1)
			seccount += 86400;              		//����2�·�����һ���������
	}

	seccount += (u32)(ReadTime->Dat - 1) * 86400; 	//��ǰ�����ڵ����������
	seccount += (u32)(ReadTime->Hor) * 3600; 		//Сʱ������
	seccount += (u32)(ReadTime->Min) * 60;			//����������
	seccount += (ReadTime->Sec);            		//�������Ӽ���ȥ

	ReadTime->Unix_Time = seccount ;
	return seccount;
}
//

u8  LED_Per[2] = {0, 0};
u8  LED_Sat[2] = {0, 0};

//LED IO��ʼ��
void LED_Init(void)
{
	RCC->AHB1ENR |= 1 << 0; //ʹ��PORTAʱ��
	GPIO_Set(GPIOA, PIN12 | PIN11, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE); //

	LED1 = 0; //LED1��
	LED2 = 0; //LED2��

	LED1 = 1; //LED1�ر�
	LED2 = 1; //LED2�ر�
}

//��50ms�ж���ˢ��LED״̬
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
* Description:    	����CRCУ��ͼ����������Ҫ�����ݱ�
* Input par:
unsigned int crcmask:    	����CRC�������ʽ����
unsigned int crcn:        	����CRCλ��,��λ:����
unsigned int ptable:     	�洢���ݱ�ָ��
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
* Description:    	CRC 16У��ͼ���.����������ݳ���4K-1 byte(2^(16-1)-1) bit).Ŀǰ���������ֽڼ����Է���ʵ��.
* Input par:
const unsigned char input:	�������ݻ���
unsigned int inputlen:    	�������ݳ���,��λ:�ֽ�
const unsigned int ptable: 	У��ͼ��������ݱ�,�ɺ���getCRCtable2()����
* Return:        	unsigned short :        	CRC 16 �����
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

