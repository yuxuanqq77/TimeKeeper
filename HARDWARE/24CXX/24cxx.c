#include "24cxx.h"
#include "delay.h"


//��ʼ��IIC�ӿ�
void AT24CXX_Init(void)
{
	IIC_Init();
}
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ
//����ֵ  :����������
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	u8 temp = 0;
	IIC_Start();

	if(EE_TYPE > AT24C16)
	{
		IIC_Send_Byte(0XA0);	   //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr >> 8); //���͸ߵ�ַ
	}
	else IIC_Send_Byte(0XA0 + ((ReadAddr / 256) << 1)); //����������ַ0XA0,д����

	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr % 256); //���͵͵�ַ
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0XA1);           //�������ģʽ
	IIC_Wait_Ack();
	temp = IIC_Read_Byte(0);
	IIC_Stop();//����һ��ֹͣ����
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
	IIC_Start();

	if(EE_TYPE > AT24C16)
	{
		IIC_Send_Byte(0XA0);	    //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr >> 8); //���͸ߵ�ַ
	}
	else IIC_Send_Byte(0XA0 + ((WriteAddr / 256) << 1)); //����������ַ0XA0,д����

	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr % 256); //���͵͵�ַ
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite);     //�����ֽ�
	IIC_Wait_Ack();
	IIC_Stop();//����һ��ֹͣ����
	delay_ms(10);
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len)
{
	u8 t;

	for(t = 0; t < Len; t++)
	{
		AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xff);
	}
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp = AT24CXX_ReadOneByte(255); //����ÿ�ο�����дAT24CXX

	if(temp == 0X55)return 0;
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255, 0X55);
		temp = AT24CXX_ReadOneByte(255);

		if(temp == 0X55)return 0;
	}

	return 1;
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
		NumToRead--;
	}
}
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
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
///  ���ò�����Flash
///
//////////////////////////////////////////////////////
int Set_Para_IIC(SYS_GSV * S_Para)
{
	SYS_Para_Save Save_Para ;

	Save_Para.DigPot		= S_Para->DigPot;             	//���ֵ�λ��ֵ��Ĭ��Ϊ469
	Save_Para.Auto_SYN		= S_Para->Auto_SYN;				//��ͬ��ʹ��
	Save_Para.Auto_OCXOADJ	= S_Para->Auto_OCXOADJ;			//�Ե�������ʹ��
	Save_Para.Prog_Pul		= S_Para->Prog_Pul;				//�ɱ������ʹ��
	Save_Para.Auto_KeepSIn	= S_Para->Auto_KeepSIn;			//���л���ʱ״̬ʹ��
	Save_Para.Meas_Pul		= S_Para->Meas_Pul;				//��������ʹ��
	Save_Para.Prog_PerT		= S_Para->Prog_PerT;			//�ɱ����������
	Save_Para.Messa_SYN		= S_Para->Messa_SYN;			//ͬ����Ϣ���ʹ��
	Save_Para.Messa_1PPS	= S_Para->Messa_1PPS;			//������������Ϣʹ��
	Save_Para.Messa_PGP		= S_Para->Messa_PGP;			//�ɱ��������Ϣʹ��
	Save_Para.Messa_GPS		= S_Para->Messa_GPS;			//GPS ����ת��ʹ��

	//STMFLASH_Write(FLASH_SAVE_ADDR ,(u32*)(&Save_Para),sizeof(SYS_Para_Save)/4);

	AT24CXX_Write(0, (u8 *)(&Save_Para), sizeof(SYS_Para_Save));

	return 1;
}
//////////////////////////////////////////////////////
///  ��ȡFlash�в����������󣬳�ʼ������
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
		Try_Para->DigPot		= 538;             		//���ֵ�λ��ֵ��Ĭ��Ϊ538
		Try_Para->Auto_SYN		= 1;					//��ͬ��ʹ��
		Try_Para->Auto_OCXOADJ	= 0;					//�Ե�������ʹ��
		Try_Para->Prog_Pul		= 0;					//�ɱ������ʹ��
		Try_Para->Auto_KeepSIn	= 0;					//���л���ʱ״̬ʹ��
		Try_Para->Meas_Pul		= 0;					//��������ʹ��
		Try_Para->Prog_PerT		= MS2ON20NS(1600) - 1;	//�ɱ����������
		Try_Para->Messa_SYN		= 1;					//ͬ����Ϣ���ʹ��
		Try_Para->Messa_1PPS	= 1;					//������������Ϣʹ��
		Try_Para->Messa_PGP		= 0;					//�ɱ��������Ϣʹ��
		Try_Para->Messa_GPS		= 0;					//GPS ����ת��ʹ��

		//STMFLASH_Write(FLASH_SAVE_ADDR ,(u32*)(Try_Para),sizeof(SYS_Para_Save)/4);
		AT24CXX_Write(0, (u8 *)(Try_Para), sizeof(SYS_Para_Save));
	}

	return 1;
}







