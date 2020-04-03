#include "TMP121.h"

//SPI2�ٶ����ú���
//SpeedSet:0~7
//SPI�ٶ�=fAPB2/2^(SpeedSet+1)
//fAPB2ʱ��һ��Ϊ84Mhz
void SPI2_SetSpeed(u8 SpeedSet)
{
	SpeedSet 	&= 0X07;			//���Ʒ�Χ
	SPI2->CR1	&= 0XFFC7;
	SPI2->CR1 	|= SpeedSet << 3;	//����SPI2�ٶ�
	SPI2->CR1 	|= 1 << 6; 			//SPI�豸ʹ��
}
//SPI2 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{
	while((SPI2->SR & 1 << 1) == 0);		//�ȴ���������

	SPI2->DR = TxData;	 	  				//����һ��byte

	while((SPI2->SR & 1 << 0) == 0);		//�ȴ�������һ��byte

	return SPI2->DR;          				//�����յ�������
}


void SPI2_Init(void)
{
	u16 tempreg = 0;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    		//ʹ��PORTBʱ��
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;   			//SPI2ʱ��ʹ��

	GPIO_Set(GPIOB, PIN12, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE);
	TMPCS = 1;

	GPIO_Set(GPIOB, PIN13 | PIN14, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);	//PB13/14���ù������
	GPIO_AF_Set(GPIOB, 13, 5);		//PB13,AF5
	GPIO_AF_Set(GPIOB, 14, 5);		//PB14,AF5

	//����ֻ���SPI�ڳ�ʼ��
	RCC->APB1RSTR |= 	  RCC_APB1RSTR_SPI2RST;			//��λSPI1
	RCC->APB1RSTR &= 	~(RCC_APB1RSTR_SPI2RST); 		//ֹͣ��λSPI1

	tempreg |= 0 << 10;				//ȫ˫��ģʽ
	tempreg |= 1 << 9;				//���nss����
	tempreg |= 1 << 8;

	tempreg |= 1 << 2;				//SPI����
	tempreg |= 0 << 11;				//8λ���ݸ�ʽ
	tempreg |= 1 << 1;				//����ģʽ��SCKΪ1 CPOL=1
	tempreg |= 1 << 0;				//���ݲ����ӵ�2��ʱ����ؿ�ʼ,CPHA=1

	//��SPI1����APB2������.ʱ��Ƶ�����Ϊ84MhzƵ��.
	tempreg |= 7 << 3;				//Fsck=Fpclk1/256
	tempreg |= 0 << 7;				//MSB First
	tempreg |= 1 << 6;				//SPI����
	SPI2->CR1 = tempreg; 			//����CR1
	SPI2->I2SCFGR &= ~(1 << 11); 	//ѡ��SPIģʽ
	SPI2_ReadWriteByte(0xff);		//��������
}
//

//��ʼ���¶ȴ�����ʹ�õ�SPI�ӿ�
void init_TMP121(void)
{
	SPI2_Init();
	SPI2_SetSpeed(4);
}
//
//��ȡ����������
s16 Get_TMP_Raw(void)
{
	u16 Res;
	u8 tmp;

	TMPCS = 0;
	Res = SPI2_ReadWriteByte(0);
	tmp = SPI2_ReadWriteByte(0);
	TMPCS = 1;

	Res = Res << 8 | (u16)tmp;
	Res = Res >> 3;
	return Res ;
}
//
