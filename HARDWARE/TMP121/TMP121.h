#ifndef __TMP121_H
#define __TMP121_H
#include "sys.h"

#define TMPCS PBout(12)

void init_TMP121(void);
s16 Get_TMP_Raw(void);

#endif
