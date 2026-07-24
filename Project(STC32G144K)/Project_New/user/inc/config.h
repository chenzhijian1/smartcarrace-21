#ifndef __CONFIG_H
#define __CONFIG_H

#include "headfile.h"

extern uint8 debug_mode;

extern float kpa;
extern float kpb;
extern float kd;
extern float kd_imu;

extern float kp_motor;
extern float ki_motor;
extern float kd_motor;

/* Kept for compatibility; the active control path uses normal_speed and s. */
extern float speed_high;
extern float speed_low;
extern float speed_90;
extern float speed_S;
extern int16 normal_speed;

void Config_Init(void);
uint8 Config_Save(void);
void Config_SetNormalSpeed(int16 speed);

/* Foreground button handling. */
void Config_ButtonInit(void);
void Config_ButtonPoll(void);

/* Legacy navigation-path ASCII storage helpers. */
float StrToDouble(const char *s);
float Config_ReadFloat(uint8 len, uint16 addr);
void eeprom_write_float_ascii(double dat, uint8 num, uint8 pointnum, uint16 addr);

#endif /* __CONFIG_H */
