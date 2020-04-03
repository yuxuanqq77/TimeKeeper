#include "timer.h"

extern SYS_GSV		G_StaV_St ;

extern u32 			PulW_NAT1P;        				//��1������20nsΪ��λ
extern u32 			PerTimer;

extern u8  			AcqTimeResFlag ;               	//���������־λ
extern u32 			AcqTime20ns ;                 	//�����ʱ��ֵ����20nsΪ��λ
extern u32 			AcqTimeS ;
extern u32 			AcqGPSTime;

extern s32 			Correct_m ;
extern u8  			AdjTimeResFlag ;

extern u8  			NaviD_Send_Flag ;

extern u8			exPulMeasFlag;
extern u32			exPul_TimerOve;				//�ⲿ���屣���������
extern u8 			exPul_TimerOve50ms;
extern u32			exPulMeas_ns;
extern u32			exPulMeas_s;
//
/////////////////////////////////////////////////////////
///   ��������TIM2��ʼ���������ź����ڡ���һ�����ȡ�
///   ͬ��ͨ��������TIM2�������ʱ����һ�������ɡ���
///   ����ͬ���ȹ���  ����50MHz @20ns
/////////////////////////////////////////////////////////
void TIM2_KEEP_Init(u32 PerTimerI, u32 PulW_NAT1PI) //
{
	RCC->APB1ENR  |=  RCC_APB1ENR_TIM2EN;     				//��TIM2��Դ
	RCC->APB1RSTR |=  RCC_APB1RSTR_TIM2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;     				//ʹ��PORTBʱ��  ��1�����ź�
	GPIO_Set(GPIOB, PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);
	GPIO_AF_Set(GPIOB, 10, 1);	  							//PB10,AF1

	TIM2->PSC = 0 ;			        						//20ns 50MHz
	TIM2->ARR = PerTimerI ;									//����

	TIM2->CCMR2 |=  TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;  	//��1�����ź�,���� T2_C3  6<<4 PWMģʽ1
	TIM2->CCMR2 |=  TIM_CCMR2_OC3PE; 	                  	//1<<3
	TIM2->CCER  |=  TIM_CCER_CC3E;   	                  	//1<<8
	TIM2->CCER  &= ~TIM_CCER_CC3P;   	                  	//0<<9	*�������Ҫ����
//	TIM2->CCER  |=  TIM_CCER_CC3P;

	TIM2->CCR3   =  PulW_NAT1PI;                          	//���ﲻ ��-1��,���õ�һ������
	TIM2->CR1   |=  TIM_CR1_ARPE;   	                  	//1<<7

	//����˿ڳ�ʼ��
	RCC->AHB1ENR  |=  RCC_AHB1ENR_GPIOAEN;      			//ʹ��PORTAʱ��
	GPIO_Set(GPIOA, PIN1, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);
	GPIO_AF_Set(GPIOA, 1, 1);	                    		//PA1,AF1

	TIM2->CCMR1 |= TIM_CCMR1_CC2S_0 ;   					//CC2S[1:0] CC2���룬ӳ�䵽TI2
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S_1 ;
	TIM2->CCMR1 |= 0 << 10 ;              					//IC2PSC[1:0]
	TIM2->CCMR1 |= 0 << 12 ;             					//IC2F

	TIM2->CCER &= ~(TIM_CCER_CC2NP);   	  					//CC2NP
	TIM2->CCER &= ~(TIM_CCER_CC2P);   	  					//CC2P  �����ز���

	TIM2->SR  &= ~(TIM_SR_CC2IF);        					//����жϱ�־λ
	TIM2->DIER &= ~(TIM_DIER_CC2IE);      					//CC2IE �����ж�
	TIM2->CCER &= ~(TIM_CCER_CC2E);   	  					//CC2E  ���򿪲���

//////////////////////////////////////////////////////
////�������ʼ���ⲿ�������IO *��������֤
	RCC->AHB1ENR  |=  RCC_AHB1ENR_GPIOBEN;      			//ʹ��PORTBʱ��
	GPIO_Set(GPIOB, PIN11, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);
	GPIO_AF_Set(GPIOB, 11, 1);	                    		//PB11,AF1

	TIM2->CCMR2 |= TIM_CCMR2_CC4S_0 ;   					//CC4S[1:0] CC4���룬ӳ�䵽TI4
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S_1 ;
//	TIM2->CCMR2 |= 0 << 10 ;              					//IC4PSC[1:0]
//	TIM2->CCMR2 |= 0 << 12 ;             					//IC4F

	TIM2->CCER &= ~(TIM_CCER_CC4NP);   	  					//CC4NP
	TIM2->CCER &= ~(TIM_CCER_CC4P);   	  					//CC4P  �����ز���

	TIM2->SR   &= ~(TIM_SR_CC4IF);        					//����жϱ�־λ
	TIM2->DIER &= ~(TIM_DIER_CC4IE);      					//CC4IE �����ж�
	TIM2->CCER &= ~(TIM_CCER_CC4E);   	  					//CC4E  ���򿪲���

////�������ʼ���ⲿ�������IO  *��������֤
////////////////////////////////////////////////////

	//���ð����ڵ���ͨ��
	TIM2->CCR1   = (PerTimerI + 1) / 2;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1CE) ;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M)  ;
	TIM2->CCMR1 |=  TIM_CCMR1_OC1PE  ;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1FE) ;
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S) ;

	MY_NVIC_Init(0, 1, TIM2_IRQn, 2);						//��ռ0�������ȼ�1����2

	TIM2->SR   &= ~(TIM_SR_UIF);          					//���жϱ�־
	TIM2->DIER |=  TIM_DIER_UIE;          					//���ж�

	TIM2->EGR |=  TIM_EGR_UG;       						//UG��1
	TIM2->CR1 |=  TIM_CR1_CEN;      						//������ʱ��
}
//���õ�һ������
void Set_PulW_NAT1P(u32 PulW_NAT1PI)
{
	PulW_NAT1P	=  PulW_NAT1PI;              				//��1������20nsΪ��λ��Ĭ��Ϊ1.25ms
	TIM2->CCR3	=  PulW_NAT1PI;              				//���ﲻ ��-1��,���õ�һ������
}
//

//���ⲿ�������
void TIM2_ExPul_Meas_Start(void)
{
	TIM2->CCER  &= ~(TIM_CCER_CC4NP);   	//CC4NP
	TIM2->CCER  &= ~(TIM_CCER_CC4P);   	  	//CC4P  �����ز���

	TIM2->SR    &= ~(TIM_SR_CC4IF);        	//����жϱ�־λ
	TIM2->DIER  |=  TIM_DIER_CC4IE;       	//CC4IE ���ж�
	TIM2->CCER  |=  TIM_CCER_CC4E;   	    //CC4E  �򿪲���

	exPulMeasFlag    	= EXPUL_STA_WAT ;
}

void TIM2_ExPul_Meas_Stop(void)
{
	TIM2->CCER  &= ~(TIM_CCER_CC4NP);   	//CC4NP
	TIM2->CCER  &= ~(TIM_CCER_CC4P);   	  	//CC4P  �����ز���

	TIM2->SR    &= ~(TIM_SR_CC4IF);        	//����жϱ�־λ

	TIM2->CCER  &= ~TIM_CCER_CC4E;   	    //CC4E  �򿪲���
	TIM2->DIER  &= ~TIM_DIER_CC4IE;       	//CC4IE ���ж�

	exPulMeasFlag    	= EXPUL_STA_OFF ;
}

///////////////////////////////////////////////////
///    ��һ�β������У������ó�ʱʱ��
///    ����ͬ��ͨ�����ã������źŲ����IO��
///////////////////////////////////////////////////
u8  Meas_OVETime = 0;

void TIM2_Meas_Start(void)
{
	TIM2->CCER  &= ~(TIM_CCER_CC2NP);   	//CC2NP
	TIM2->CCER  &= ~(TIM_CCER_CC2P);   	  	//CC2P  �����ز���

	TIM2->SR    &= ~(TIM_SR_CC2IF);        	//����жϱ�־λ
	TIM2->DIER  |=  TIM_DIER_CC2IE;       	//CC2IE ���ж�
	TIM2->CCER  |=  TIM_CCER_CC2E;   	    //CC2E  �򿪲���

	//���ó�ʱʱ��50*30=1.5s ���ǵ����ص�Ƭ������һ��
	//GPS��������֮ǰ1s�ڷ���ͬ�������ָ���˸ó�ʱ����Ϊ����2s
	Meas_OVETime = 30 ;
}
//

///////////////////////////////////////////////////
///    ��һ�ε������У������ó�ʱʱ��
///    ����ͬ��ͨ�����ã������źŲ����IO��
///////////////////////////////////////////////////
u16 Adj_OVETime = 0 ;

void TIM2_Adj_Start(void)
{
	TIM2->CCER  &=	~(TIM_CCER_CC1NP);   	   	//CC1NP
	TIM2->CCER  &=	~(TIM_CCER_CC1P);   	   	//CC1P
	TIM2->CCMR1 |=	 0x06 << 4;                	//PWM1,OC1M  = 110
	TIM2->SR    &=	~(TIM_SR_CC1IF);         	//����жϱ�־λ
	TIM2->DIER  |=	  TIM_DIER_CC1IE;     		//CC1IE ���ж�

	//���ó�ʱʱ�� 50*30=1.5s��ʱ
	Adj_OVETime = 30 ;
}
//

//��������ĵ�������������ʾ�����۲�õ�
s32 Adj_SoftC = 13 ;

//��ʱ��2�жϷ������
//�����������ĺ���
//���ݵõ�����ֵ
//���ݲ���ֵ����TCNT
//����ط��߼��ǳ�����,�����ڱ߽�����,����С����֤���и�visio���ڹ����ļ����
void TIM2_IRQHandler(void)
{
	//ֻ������3���жϲ���
	//1.����ͬ������
	//2.��������������ж�
	//3.�������жϣ����TCNT����
	//4.�ⲿ�������岶���жϣ����ڲ����ⲿ����ʱ��

	//����� �����ж� �� �����ж�
	if(TIM2->SR & TIM_SR_CC2IF)	//�����ж�
	{
		if(TIM2->DIER & TIM_DIER_CC2IE)
		{
			AcqTime20ns			= TIM2->CCR2 ;            				//�Ѳ���Ľ��ȡ����
			AcqTimeS			= G_StaV_St.NATI_Time ;

			AcqTimeResFlag    	= 1 ;                     				//���ñ�־λ����������֪���������

			TIM2->CCER  &= ~(TIM_CCER_CC2E);   	        				//CC2E  �رղ���
			TIM2->DIER  &= ~(TIM_DIER_CC2IE);            				//CC2IE �ر��ж�
			TIM2->SR    &= ~(TIM_SR_CC2IF);              				//����жϱ�־λ

			if( AcqTime20ns < 50000 )									//���粶��Ľ��С��1ms�������п�����Ҫ�Բ����s����������
			{
				if(TIM2->SR	& TIM_SR_UIF)								//�����Ѿ������˸����жϣ�����û�д���
				{
					AcqTimeS++ ;

					//����������
//����ط����ܸ���ʱ�䣬���ܻ�Ӱ�쵽 �ⲿ���岶���ж�
//					G_StaV_St.NATI_Time++ ;								//˳���ְѱ���������Ҳ������������
//					TIM2->SR    &=	~(TIM_SR_UIF);            			//����жϱ�־λ
				}
				else
				{
					//�ܵ��������Ϊ���������Ȼ�Ƚ�С�����ǣ������Ѿ���Ӧ���ˣ�ȡ����ns��s������ȷ��
				}
			}
		}
		else
		{
			//������������ܵ���������������ˣ����ǰ��ж����˱Ƚϰ�ȫ
			TIM2->CCER  &= ~(TIM_CCER_CC2E);   	        				//CC2E  �رղ���
			TIM2->DIER  &= ~(TIM_DIER_CC2IE);            				//CC2IE �ر��ж�
			TIM2->SR    &= ~(TIM_SR_CC2IF);              				//����жϱ�־λ
		}
	}
	else if(TIM2->SR & TIM_SR_CC4IF)									//4.�ⲿ�������岶���жϣ����ڲ����ⲿ����ʱ��
	{
		if(TIM2->DIER & TIM_DIER_CC4IE)
		{
			exPulMeas_ns		= TIM2->CCR4 ;            				//�Ѳ���Ľ��ȡ����
			exPulMeas_s			= G_StaV_St.NATI_Time ;

			TIM2->CCER  &= ~(TIM_CCER_CC4E);   	        				//CC4E  �رղ���
			TIM2->DIER  &= ~(TIM_DIER_CC4IE);            				//CC4IE �ر��ж�
			TIM2->SR    &= ~(TIM_SR_CC4IF);              				//����жϱ�־λ �ص���Ϊ��ʵ�ּ������

			if( exPulMeas_ns < 50000 )									//���粶��Ľ��С��1ms�������п�����Ҫ�Բ����s����������
			{
				if(TIM2->SR	& TIM_SR_UIF)								//�����Ѿ������˸����жϣ�����û�д���
				{
					exPulMeas_s++ ;
//����ط����ܸ���ʱ�䣬���ܻ�Ӱ�쵽ͬ�������ж�
//					G_StaV_St.NATI_Time++ ;								//˳���ְѱ���������Ҳ������������
//					TIM2->SR    &=	~(TIM_SR_UIF);            			//����жϱ�־λ
				}
				else
				{
					//�ܵ��������Ϊ���������Ȼ�Ƚ�С�����ǣ������Ѿ���Ӧ���ˣ�ȡ����ns��s������ȷ��
				}
			}

			exPulMeasFlag    	=	EXPUL_STA_IED ;                   	//���ñ�־λ����������֪���������
			exPul_TimerOve50ms	=	G_StaV_St.RunTime_50ms + 3;			//���ñ������100-150ms
			exPul_TimerOve		=	G_StaV_St.RunTime + exPul_TimerOve50ms / 20 ;
			exPul_TimerOve50ms	=	exPul_TimerOve50ms % 20 ;
		}
		else
		{
			//������������ܵ���������������ˣ����ǰ��ж����˱Ƚϰ�ȫ
			TIM2->CCER  &= ~(TIM_CCER_CC4E);   	        				//CC4E  �رղ���
			TIM2->DIER  &= ~(TIM_DIER_CC4IE);            				//CC4IE �ر��ж�
			TIM2->SR    &= ~(TIM_SR_CC4IF);              				//����жϱ�־λ
		}
	}
	else if(TIM2->SR & TIM_SR_UIF)										//����s�����ж�
	{
		//����������
		G_StaV_St.NATI_Time++ ;
		TIM2->SR    &= ~(TIM_SR_UIF);            						//����жϱ�־λ

		if(G_StaV_St.Messa_1PPS == 1)									//��λ���������屨��
			NaviD_Send_Flag = 1 ;
		else
			NaviD_Send_Flag = 0 ;
	}

	//3.�������жϱȽϼ�,��Ϊ���Լ���һ��,��������ռ�ķ���,����,���࿼��
	//�������ǵ���������ǰ�غ�GPS������ǰ�ض������Ҫ����
	//�����ڱȽ��ж����ڵ���TCNT
	if(TIM2->SR & TIM_SR_CC1IF )
	{
		if( TIM2->DIER & TIM_DIER_CC1IE )
		{
			//����ǰ������ж�
			Correct_m += Adj_SoftC ;
			TIM2->CNT += Correct_m ;              		//����
			Correct_m -= Adj_SoftC ;

			if(AcqGPSTime != G_StaV_St.NATI_Time)
			{
				G_StaV_St.NATI_Time = AcqGPSTime ;
			}

			AdjTimeResFlag = 1 ;       					//���õ�����־λ����������֪���������

			TIM2->CCER	&=	~(TIM_CCER_CC1E);   	    //CC1E  �ر�
			TIM2->DIER	&=	~(TIM_DIER_CC1IE);      	//CC1IE �ر�
			TIM2->SR  	&=	~(TIM_SR_CC1IF);     		//����жϱ�־λ
		}
		else
		{
			//������������ܵ���������������ˣ����ǰ��ж����˱Ƚϰ�ȫ
			TIM2->CCER	&=	~(TIM_CCER_CC1E);   	    //CC1E  �ر�
			TIM2->DIER	&=	~(TIM_DIER_CC1IE);      	//CC1IE �ر�
			TIM2->SR  	&=	~(TIM_SR_CC1IF);     		//����жϱ�־λ
		}
	}
}
//

//��ʱ��2�ֶ��رպ��������ڲ��ԣ�����ʧ��
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
//��ʱ��6,��������ʱ��RunTimer����
//ÿ50ms�ж�һ�Σ�20�μ�1s
////////////////////////////////////////////////
void TIM6_50ms_Init(void)
{
	RCC->APB1ENR 	|=  RCC_APB1ENR_TIM6EN ;
	RCC->APB1RSTR	|=  RCC_APB1RSTR_TIM6RST;
	RCC->APB1RSTR	&= ~(RCC_APB1RSTR_TIM6RST);

	TIM6->PSC  = 2000 - 1 ;
	TIM6->ARR  = 1250 - 1 ;
	TIM6->CR1 |= TIM_CR1_ARPE ;

	MY_NVIC_Init(2, 3, TIM6_DAC_IRQn, 2);	//��ռ2�������ȼ�3����2

	TIM6->EGR  |=  TIM_EGR_UG;            	//UG��1
	TIM6->SR   &= ~(TIM_SR_UIF);          	//���жϱ�־
	TIM6->DIER |=  TIM_DIER_UIE;          	//���ж�

	TIM6->CR1  |=  TIM_CR1_CEN;           	//��������
}
//

void TIM6_DAC_IRQHandler(void)
{
	if(TIM6->SR & TIM_SR_UIF  &&  TIM6->DIER & TIM_DIER_UIE ) 			//�ж�
	{
		TIM6->SR   &= ~(TIM_SR_UIF);                             		//���жϱ�־

		G_StaV_St.RunTime_50ms++ ;
		G_StaV_St.RunTime_50ms %= 20;

		if(G_StaV_St.RunTime_50ms == 0)
			G_StaV_St.RunTime++;

		LED_ReFresh();								//����LED��ʾ

		if( G_StaV_St.RunTime_50ms % 10 == 0)
		{
//			ReFresh_ADC();							//ˢ��ADC�ɼ����������ֻ����STM�ڲ��¶ȴ����� *�˴���ע�ͣ�ʹ���ⲿ�¶ȴ�����
		}
	}

	//������ʱ�ж�
	if(Meas_OVETime != 0)
	{
		Meas_OVETime--;

		if(Meas_OVETime == 0)
		{
			//��������������ж�
			TIM2->CCER &= ~(TIM_CCER_CC2E);   	  	//CC2E  �رղ���
			TIM2->DIER &= ~(TIM_DIER_CC2IE);		//CC2IE �ر��ж�
			TIM2->SR  &= ~(TIM_SR_CC2IF);        	//����жϱ�־λ

			AcqTimeResFlag = 2 ;              		//���ó�ʱ��־λ����������֪��������ʱ
		}
	}

	if(Adj_OVETime != 0)
	{
		Adj_OVETime--;

		if(Adj_OVETime == 0)
		{
			//������رհ������ж�
			TIM2->CCER	&= ~(TIM_CCER_CC1E);   			//CC1E  �ر�
			TIM2->DIER 	&= ~(TIM_DIER_CC1IE);       	//CC1IE �ر�
			TIM2->SR	&= ~(TIM_SR_CC1IF);         	//����жϱ�־λ

			AdjTimeResFlag  = 2 ;               		//���ó�ʱ��־λ����������֪��������ʱ
		}
	}
}
//
//��ʱ��麯����ͨ��
//����Ҫ�����ʱ�����뺯��
//������ڵ�ʱ�䳬�������ʱ�䣬�ͷ��س�ʱ1
//������ڵ�ʱ�仹û�е��������ʱ�䣬����û�г�ʱ0
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

//�������������
//�������ڡ���1�����ȣ�����ʱ�ӣ�������Ҫ����TCNT����ֵ������
//�����з�������ֱ����TCNT��Ӽ���
//�����㷨�μ������������,���бʼǱ�P123/124.
//
s32 AdjCalc(u32 PerT, u32 AcqD, u32 PulW_NAT1P)
{
	u32 CPot, DPot ;                           			//C,DΪ����������ֵ��,���ǵ�������Ϊ�˱�֤,�����ڵ�����ʱ��
	//������Ϊ�������ȹ�������������1��������,��ʱ�����
	//�ȴ������״̬

	u32 Tn ;                                  			//�޷��Ų���ֵ����任,��AcqD���SYN�źŷ�����TIM2�����е�λ��
	//�任����1����ǰ��,������SYN�����е�λ��.

	s32 Corrm;                                			//�з�������ֵ,���������յĵ���ֵ,��ֵ������ֵ������

	//CPot = (PerT + 1)/2 - PulW_NAT1P - 500000 ; 		//500000 ��Ӧ10ms�������� ����C��λ��
	//DPot = (PerT + 1)/2 - 1        + 500000 ; 		//500000 ��Ӧ10ms�������� ����D��λ��

	CPot = (PerT + 1) / 2                 	- 500000 ; 	//500000 ��Ӧ10ms�������� ����C��λ��
	DPot = (PerT + 1) / 2 - 1 + PulW_NAT1P  + 500000 ; 	//500000 ��Ӧ10ms�������� ����D��λ��

	Tn   = (PerT - AcqD + 1) % (PerT + 1) ;     		//����Tn,ע����AcqD=0,�п����������ʹ��%

	if( Tn < CPot )                           			//0<Tn<C,��1����ǰ�ط�����SYN�����е�ǰ�벿����û�дﵽ��ֵ
	{
		//˵��FR�ź������SYNһС��,FR����,��Ҫ��FR�źſ�һ�㵽,TCNTҪ"+"
		Corrm = Tn ;
	}
	else if( Tn <= (PerT + 1) / 2 )             		//C<=Tn<=(PerT + 1)/2,��1����ǰ�ط�����SYN�����е�ǰ�벿���Ҵﵽ��ֵ
	{
		//˵��FR�ź������SYNһ���,FR����,��Ҫ��FR�źſ�һ�㵽,TCNTҪ"+",
		Corrm = CPot ;                          		//�����ֲ��ܼ�̫��,��ֹ�Ӷ���,��������1��,������.
	}
	else if( Tn <= DPot )                     			//(PerT + 1)/2<Tn<=D,��1����ǰ�ط�����SYN�����еĺ�벿��,������ֵ֮ǰ(������������ֵ)
	{
		//˵��FR�źų�ǰ��SYNһ���,FR����,��Ҫ��FR�ź���һ�㵽,TCNTҪ"-",
		Corrm = -(PerT - DPot + 1) ;           			//�����ֲ��ܼ�̫��,��ֹ:1,��������1��,��ǰ���;2,��TCNT�䵽��һ�����ڼ�,�������ɵ�һ�������.
	}
	else                                      			//D<Tn<=N,��1����ǰ�ط�����SYN�����еĺ�벿��,������ֵ֮��(������û�г�����ֵ)
	{
		//˵��FR�źų�ǰ��SYNһС��,FR����,��Ҫ��FR�ź���һ�㵽,TCNTҪ"-",
		Corrm = -(PerT - Tn   + 1) ;            		//��Ϊ��һС��,����ֱ�Ӽ�ȥ�����,���ᷢ������.
	}

	return Corrm ;                            			//�������ֵ,�����и�,������ֵ�޷�
}
//

