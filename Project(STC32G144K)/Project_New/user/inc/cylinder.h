#ifndef __CYLINDER_H_
#define __CYLINDER_H_

#include "zf_common_typedef.h"

#define CYLINDER_ENTRY_PITCH_MAX_DEG          (-30.0f)
#define CYLINDER_ENTRY_CONFIRM_SAMPLES         (3U)

#define CYLINDER_STATE_IDLE                    (0U)
#define CYLINDER_STATE_DETECTED                (1U)

void Cylinder_ImuUpdate(float pitch_deg);
uint8 Cylinder_HasExited(void);
uint8 Cylinder_GetState(void);
void Cylinder_Reset(void);

#endif
