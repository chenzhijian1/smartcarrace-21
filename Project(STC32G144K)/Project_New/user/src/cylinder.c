#include "cylinder.h"

static volatile uint8 cylinder_state = CYLINDER_STATE_IDLE;
static uint8 cylinder_entry_count = 0;

void Cylinder_ImuUpdate(float pitch_deg)
{
    if (cylinder_state != CYLINDER_STATE_IDLE)
        return;

    if (pitch_deg < CYLINDER_ENTRY_PITCH_MAX_DEG)
    {
        if (cylinder_entry_count < CYLINDER_ENTRY_CONFIRM_SAMPLES)
            cylinder_entry_count++;
    }
    else
    {
        cylinder_entry_count = 0;
    }

    if (cylinder_entry_count >= CYLINDER_ENTRY_CONFIRM_SAMPLES)
    {
        cylinder_entry_count = 0;
        cylinder_state = CYLINDER_STATE_DETECTED;
    }
}

uint8 Cylinder_HasExited(void)
{
    return (uint8)(cylinder_state == CYLINDER_STATE_DETECTED);
}

uint8 Cylinder_GetState(void)
{
    return cylinder_state;
}

void Cylinder_Reset(void)
{
    cylinder_entry_count = 0;
    cylinder_state = CYLINDER_STATE_IDLE;
}
