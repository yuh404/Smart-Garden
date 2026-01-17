#ifndef __BH1750_H
#define __BH1750_H

#include "stm32f10x.h"

#define BH1750_ADDR_LOW   0x23   // ADDR = LOW
#define BH1750_ADDR_HIGH  0x5C   // ADDR = HIGH

void I2C2_Init(void);
void BH1750_Init(void);
float BH1750_ReadLux(void);


#endif