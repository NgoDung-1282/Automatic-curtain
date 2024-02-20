#ifndef __DS1307_H__
#define __DS1307_H__

#include "main.h"
void RTC_init(I2C_HandleTypeDef *hi2c_init);
void RTC_Write_Time(uint8_t sec, uint8_t min, uint8_t hour);
void RTC_Write_Date(uint8_t day, uint8_t month, uint8_t year);
void RTC_Read_Time(uint8_t *sec, uint8_t *min, uint8_t *hour);
void RTC_Read_Date(uint8_t *day, uint8_t *month, uint8_t *year);
#endif
