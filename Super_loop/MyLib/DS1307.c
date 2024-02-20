#include "DS1307.h"

uint8_t BCD2Decimal(uint8_t num)
	{
		return (num>>4)*10+(num&0x0F);
	}
uint8_t Decimal2BCD(uint8_t num)
	{
		return (num/10)<<4|(num%10);
	}

extern I2C_HandleTypeDef hi2c1;
#define DS1307_address (0x68 << 1)
//--------------------------------Write-Time-----------------------------------
void RTC_Write_Time(uint8_t sec, uint8_t min, uint8_t hour)
	{
		uint8_t Data_time_Set[] = {Decimal2BCD(sec), Decimal2BCD(min), Decimal2BCD(hour)};
		HAL_I2C_Mem_Write(&hi2c1, DS1307_address, 0x00, I2C_MEMADD_SIZE_8BIT, Data_time_Set, 3, 100);
	}
void RTC_Write_Date(uint8_t day, uint8_t month, uint8_t year)
	{
		uint8_t Data_date_Set[] = {Decimal2BCD(day), Decimal2BCD(month), Decimal2BCD(year)};
		HAL_I2C_Mem_Write(&hi2c1, DS1307_address, 0x04, I2C_MEMADD_SIZE_8BIT, Data_date_Set, 3, 100);
	}
	
//--------------------------------Read-Time-------------------------------------
void RTC_Read_Time(uint8_t *sec, uint8_t *min, uint8_t *hour)
	{
		uint8_t Data_time_Get[3];
		HAL_I2C_Mem_Read(&hi2c1, DS1307_address, 0x00, I2C_MEMADD_SIZE_8BIT, Data_time_Get, 3, 100);
	  *sec = BCD2Decimal(Data_time_Get[0]);
		*min = BCD2Decimal(Data_time_Get[1]);
		*hour = BCD2Decimal(Data_time_Get[2]);	
	}
void RTC_Read_Date(uint8_t *day, uint8_t *month, uint8_t *year)
	{
		uint8_t Data_date_Get[3];
		HAL_I2C_Mem_Read(&hi2c1, DS1307_address, 0x04, I2C_MEMADD_SIZE_8BIT, Data_date_Get, 3, 100);
	  *day = BCD2Decimal(Data_date_Get[0]);
		*month = BCD2Decimal(Data_date_Get[1]);
		*year = BCD2Decimal((Data_date_Get[2]));		
	}

