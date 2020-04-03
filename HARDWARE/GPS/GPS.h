#ifndef __GPS_H
#define __GPS_H
#include "sys.h"
#include "stdio.h"

#define UARTRXBUFLEN_U1 1024

u8 Proc_RxCMD_GPS(u8 * DataBuf);
u16 Proc_RxByte_GPS(void);
void UART1_GPS_init(u32 pclk2, u32 bound);

void Prorc_RNSS_GPS_RMC_GSV( u8 * RNSS_GPSData );
void GPS_Pow_init(void);
void GPS_Pow(u8 OnOff);
void GPS_Pow_Re(void);
void Prorc_GPS_Top(void);

#endif

