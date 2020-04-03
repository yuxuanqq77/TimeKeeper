#include "DIGPOT.h"
#include "spi.h"
#include "delay.h"

//控制恒温晶振压控电压的数字电位器SPI初始化
void DIGPOT_Init(void)
{
	RCC->AHB1ENR |= 1 << 3; //使能PORTD时钟
	DIGPOT = 1;
	GPIO_Set(GPIOD, PIN2, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_PU);	//
	DIGPOT = 1;
	SPI1_Init();
	SPI1_SetSpeed(SPI_SPEED_8);
}
//写数字电阻器
void DIGPOT_Write(u16 DigVal)
{
	u8 TempU1, TempU2 ;

	if(DigVal > 0x3FF)
		DigVal = 0x3FF;

	DigVal = DigVal << 6 ;

	TempU1 = (u8)((DigVal & 0xFF00) >> 8) ;
	TempU2 = (u8)( DigVal & 0x00FF) ;

	DIGPOT = 0;
	SPI1_ReadWriteByte(0x00);
	SPI1_ReadWriteByte(TempU1);
	SPI1_ReadWriteByte(TempU2);
	DIGPOT = 1;

}
//
