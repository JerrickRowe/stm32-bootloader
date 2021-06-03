#ifndef __BSP_POWER_H__
#define __BSP_POWER_H__

#include <stdbool.h>

void bsp_power_HoldPower(void);

void bsp_power_ReleasePower(void);

float bsp_power_GetExtPowerVoltage(void);

bool bsp_power_isExtPowerOnline(void);

#endif
