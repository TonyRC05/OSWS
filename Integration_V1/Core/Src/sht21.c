
#include "sht21.h"
#include "main.h"
#ifdef __cplusplus
extern "C"{
#endif

//typedef enum {
//	STATUS_OK = 1,
//	STATUS_FAIL = 0,
//}Status_Flag;

I2C_HandleTypeDef *_sht2x_ui2c;

Status_Flag SHT2x_Init(I2C_HandleTypeDef *hi2c) {
	_sht2x_ui2c = hi2c;
	return STATUS_OK;
}

void SHT2x_SoftReset(void){
	uint8_t cmd = SHT2x_SOFT_RESET;
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &cmd, 1, SHT2x_TIMEOUT);
}

uint8_t SHT2x_ReadUserReg(void) {
	uint8_t val;
	uint8_t cmd = SHT2x_READ_REG;
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &cmd, 1, SHT2x_TIMEOUT);
	HAL_I2C_Master_Receive(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &val, 1, SHT2x_TIMEOUT);
	return val;
}

uint16_t SHT2x_GetRaw(uint8_t cmd) {
	uint8_t val[3] = { 0 };
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &cmd, 1, SHT2x_TIMEOUT);
	HAL_I2C_Master_Receive(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, val, 3, SHT2x_TIMEOUT);
	return val[0] << 8 | val[1];
}

float SHT2x_GetTemperature() {
	uint8_t cmd = SHT2x_READ_TEMP_HOLD;
	return -46.85 + 175.72 * (SHT2x_GetRaw(cmd) / 65536.0);
}

float SHT2x_GetRelativeHumidity() {
	uint8_t cmd = SHT2x_READ_RH_HOLD;
	return -6 + 125.00 * (SHT2x_GetRaw(cmd) / 65536.0);
}

void SHT2x_SetResolution(SHT2x_Resolution res) {
	uint8_t val = SHT2x_ReadUserReg();
	val = (val & 0x7e) | res; //Write on bit 0 and 7
	uint8_t temp[2] = { SHT2x_WRITE_REG, val }; //An array of commands
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, temp, 2, SHT2x_TIMEOUT);
}
