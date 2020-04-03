#ifndef __DIGPOT_H
#define __DIGPOT_H
#include "spi.h"

#define DIGPOT PDout(2)

void DIGPOT_Init(void);
void DIGPOT_Write(u16 DigVal);

#endif
