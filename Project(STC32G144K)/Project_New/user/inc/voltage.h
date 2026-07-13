#ifndef __VOLTAGE_H
#define __VOLTAGE_H

#include "zf_common_typedef.h"

#define VOLTAGE_LOW_THRESHOLD_MV 10800UL

uint16 voltage_battery_get_mv(void);
uint8 voltage_battery_is_low(void);

#endif
