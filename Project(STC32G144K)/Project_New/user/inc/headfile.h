#ifndef __USER_HEADFILE_H_
#define __USER_HEADFILE_H_

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "stc32g144k246.h"
#include "intrins.h"

extern uint8 busy[9];

// ---- User modules ----
#include "my_motor.h"
#include "inductance.h"
#include "my_peripheral.h"
#include "navigation.h"
#include "car_control.h"
#include "huandao.h"
#include "config.h"
#include "quaternion.h"
#include "ui_menu.h"
#include "voltage.h"

#endif
