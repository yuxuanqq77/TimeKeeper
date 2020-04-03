#include "TMP121.h"

//SPI2速度设置函数
//SpeedSet:0~7
//SPI速度=fAPB2/2^(SpeedSet+1)
//fAPB2时钟一般为84Mhz
void SPI2_SetSpeed(u8 SpeedSet)
{
	SpeedSet 	&= 0X07;			//限制范围
	SPI2->CR1	&= 0XFFC7;
	SPI2->CR1 	|= SpeedSet << 3;	//设置SPI2速度
	SPI2->CR1 	|= 1 << 6; 			//SPI设备使能
}
//SPI2 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{
	while((SPI2->SR & 1 << 1) == 0);		//等待发送区空

	SPI2->DR = TxData;	 	  				//发送一个byte

	while((SPI2->SR & 1 << 0) == 0);		//等待接收完一个byte

	return SPI2->DR;          				//返回收到的数据
}


void SPI2_Init(void)
{
	u16 tempreg = 0;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    		//使能PORTB时钟
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;   			//SPI2时钟使能

	GPIO_Set(GPIOB, PIN12, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_2M, GPIO_PUPD_NONE);
	TMPCS = 1;

	GPIO_Set(GPIOB, PIN13 | PIN14, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_NONE);	//PB13/14复用功能输出
	GPIO_AF_Set(GPIOB, 13, 5);		//PB13,AF5
	GPIO_AF_Set(GPIOB, 14, 5);		//PB14,AF5

	//这里只针对SPI口初始化
	RCC->APB1RSTR |= 	  RCC_APB1RSTR_SPI2RST;			//复位SPI1
	RCC->APB1RSTR &= 	~(RCC_APB1RSTR_SPI2RST); 		//停止复位SPI1

	tempreg |= 0 << 10;				//全双工模式
	tempreg |= 1 << 9;				//软件nss管理
	tempreg |= 1 << 8;

	tempreg |= 1 << 2;				//SPI主机
	tempreg |= 0 << 11;				//8位数据格式
	tempreg |= 1 << 1;				//空闲模式下SCK为1 CPOL=1
	tempreg |= 1 << 0;				//数据采样从第2个时间边沿开始,CPHA=1

	//对SPI1属于APB2的外设.时钟频率最大为84Mhz频率.
	tempreg |= 7 << 3;				//Fsck=Fpclk1/256
	tempreg |= 0 << 7;				//MSB First
	tempreg |= 1 << 6;				//SPI启动
	SPI2->CR1 = tempreg; 			//设置CR1
	SPI2->I2SCFGR &= ~(1 << 11); 	//选择SPI模式
	SPI2_ReadWriteByte(0xff);		//启动传输
}
//

//初始化温度传感器使用的SPI接口
void init_TMP121(void)
{
	SPI2_Init();
	SPI2_SetSpeed(4);
}
//
//获取传感器数据
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
