#include "main.h"
#include "stdbool.h"

#ifndef SHT2X_FOR_STM32_HAL_H
#define SHT2X_FOR_STM32_HAL_H
#endif

#define SHT2x_I2C_ADDR			0x40
#define SHT2x_HOLD_MASTER		1
#define SHT2x_READ_TEMP_HOLD	0xe3
#define	SHT2x_READ_RH_HOLD		0xe5
#define SHT2x_READ_TEMP_NOHOLD	0xf3
#define SHT2x_READ_RH_NOHOLD	0xf5
#define	SHT2x_WRITE_REG			0xe6
#define SHT2x_READ_REG			0xe7
#define SHT2x_SOFT_RESET		0xfe
#define SHT2x_TIMEOUT			1000

typedef enum SHT2x_Resolution {
	RES_14_12 = 0x00,
	RES_12_8 = 0x01,
	RES_13_10 = 0x80,
	RES_11_11 = 0x81,
} SHT2x_Resolution;



//I2C_HandleTypeDef *_sht2x_ui2c;

Status_Flag SHT2x_Init(I2C_HandleTypeDef *hi2c);
void SHT2x_SoftReset(void);
uint8_t SHT2x_ReadUserReg(void);
uint16_t SHT2x_GetRaw(uint8_t cmd);
float SHT2x_GetTemperature();
float SHT2x_GetRelativeHumidity();



