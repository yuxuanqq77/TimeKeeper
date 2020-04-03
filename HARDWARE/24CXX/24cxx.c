#include "24cxx.h"
#include "delay.h"


//初始化IIC接口
void AT24CXX_Init(void)
{
	IIC_Init();
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址
//返回值  :读到的数据
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	u8 temp = 0;
	IIC_Start();

	if(EE_TYPE > AT24C16)
	{
		IIC_Send_Byte(0XA0);	   //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr >> 8); //发送高地址
	}
	else IIC_Send_Byte(0XA0 + ((ReadAddr / 256) << 1)); //发送器件地址0XA0,写数据

	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr % 256); //发送低地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XA1);           //进入接收模式
	IIC_Wait_Ack();
	temp = IIC_Read_Byte(0);
	IIC_Stop();//产生一个停止条件
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
	IIC_Start();

	if(EE_TYPE > AT24C16)
	{
		IIC_Send_Byte(0XA0);	    //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr >> 8); //发送高地址
	}
	else IIC_Send_Byte(0XA0 + ((WriteAddr / 256) << 1)); //发送器件地址0XA0,写数据

	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr % 256); //发送低地址
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite);     //发送字节
	IIC_Wait_Ack();
	IIC_Stop();//产生一个停止条件
	delay_ms(10);
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len)
{
	u8 t;

	for(t = 0; t < Len; t++)
	{
		AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xff);
	}
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 AT24CXX_ReadLenByte(u16 ReadAddr, u8 Len)
{
	u8 t;
	u32 temp = 0;

	for(t = 0; t < Len; t++)
	{
		temp <<= 8;
		temp += AT24CXX_ReadOneByte(ReadAddr + Len - t - 1);
	}

	return temp;
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp = AT24CXX_ReadOneByte(255); //避免每次开机都写AT24CXX

	if(temp == 0X55)return 0;
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255, 0X55);
		temp = AT24CXX_ReadOneByte(255);

		if(temp == 0X55)return 0;
	}

	return 1;
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
		NumToRead--;
	}
}
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}


//////////////////////////////////////////////////////
///  设置参数到Flash
///
//////////////////////////////////////////////////////
int Set_Para_IIC(SYS_GSV * S_Para)
{
	SYS_Para_Save Save_Para ;

	Save_Para.DigPot		= S_Para->DigPot;             	//数字电位器值，默认为469
	Save_Para.Auto_SYN		= S_Para->Auto_SYN;				//自同步使能
	Save_Para.Auto_OCXOADJ	= S_Para->Auto_OCXOADJ;			//自调整晶振使能
	Save_Para.Prog_Pul		= S_Para->Prog_Pul;				//可编程脉冲使能
	Save_Para.Auto_KeepSIn	= S_Para->Auto_KeepSIn;			//自切换守时状态使能
	Save_Para.Meas_Pul		= S_Para->Meas_Pul;				//测量功能使能
	Save_Para.Prog_PerT		= S_Para->Prog_PerT;			//可编程脉冲周期
	Save_Para.Messa_SYN		= S_Para->Messa_SYN;			//同步信息输出使能
	Save_Para.Messa_1PPS	= S_Para->Messa_1PPS;			//本机秒脉冲信息使能
	Save_Para.Messa_PGP		= S_Para->Messa_PGP;			//可编程脉冲信息使能
	Save_Para.Messa_GPS		= S_Para->Messa_GPS;			//GPS 报文转发使能

	//STMFLASH_Write(FLASH_SAVE_ADDR ,(u32*)(&Save_Para),sizeof(SYS_Para_Save)/4);

	AT24CXX_Write(0, (u8 *)(&Save_Para), sizeof(SYS_Para_Save));

	return 1;
}
//////////////////////////////////////////////////////
///  获取Flash中参数，若有误，初始化参数
///
//////////////////////////////////////////////////////
int Get_Para_IIC(SYS_Para_Save * Try_Para)
{

	//STMFLASH_Read(FLASH_SAVE_ADDR ,(u32*)(Try_Para),sizeof(SYS_Para_Save)/4);

	AT24CXX_Read(0, (u8 *)(Try_Para), sizeof(SYS_Para_Save));

	if(	(Try_Para->DigPot 		<	10 	|| Try_Para->DigPot 		> 1000)
			||	(Try_Para->Auto_SYN 	!= 	0 	&& Try_Para->Auto_SYN 		!= 1)
			||	(Try_Para->Auto_OCXOADJ	!= 	0 	&& Try_Para->Auto_OCXOADJ 	!= 1)
			||	(Try_Para->Prog_Pul 	!= 	0 	&& Try_Para->Prog_Pul 		!= 1)
			||	(Try_Para->Auto_KeepSIn	!= 	0 	&& Try_Para->Auto_KeepSIn 	!= 1)
			||	(Try_Para->Meas_Pul		!= 	0 	&& Try_Para->Meas_Pul 		!= 1)
			||	(Try_Para->Messa_SYN	!= 	0 	&& Try_Para->Messa_SYN 		!= 1)
			||	(Try_Para->Messa_1PPS 	!= 	0 	&& Try_Para->Messa_1PPS 	!= 1)
			||	(Try_Para->Messa_PGP	!= 	0 	&& Try_Para->Messa_PGP 		!= 1)
			||	(Try_Para->Messa_GPS	!= 	0 	&& Try_Para->Messa_GPS 		!= 1)
	  )
	{
		Try_Para->DigPot		= 538;             		//数字电位器值，默认为538
		Try_Para->Auto_SYN		= 1;					//自同步使能
		Try_Para->Auto_OCXOADJ	= 0;					//自调整晶振使能
		Try_Para->Prog_Pul		= 0;					//可编程脉冲使能
		Try_Para->Auto_KeepSIn	= 0;					//自切换守时状态使能
		Try_Para->Meas_Pul		= 0;					//测量功能使能
		Try_Para->Prog_PerT		= MS2ON20NS(1600) - 1;	//可编程脉冲周期
		Try_Para->Messa_SYN		= 1;					//同步信息输出使能
		Try_Para->Messa_1PPS	= 1;					//本机秒脉冲信息使能
		Try_Para->Messa_PGP		= 0;					//可编程脉冲信息使能
		Try_Para->Messa_GPS		= 0;					//GPS 报文转发使能

		//STMFLASH_Write(FLASH_SAVE_ADDR ,(u32*)(Try_Para),sizeof(SYS_Para_Save)/4);
		AT24CXX_Write(0, (u8 *)(Try_Para), sizeof(SYS_Para_Save));
	}

	return 1;
}







