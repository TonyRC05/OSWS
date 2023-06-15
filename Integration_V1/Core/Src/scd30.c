#include "scd30.h"
#include "main.h"
//
//I2C_HandleTypeDef * scd30_i2c;
//
//Status_Flag SCD30_Init(I2C_HandleTypeDef *hi2c){
//	uint8_t cmd[2]={FIRMWARE_MSB, FIRMWARE_LSB};
//	uint8_t value[3]={0};
//	scd30_i2c=hi2c;
//	HAL_I2C_Master_Transmit(scd30_i2c, SCD30_I2C_ADDR, cmd, 2, 1000);
//	HAL_I2C_Master_Receive(scd30_i2c, SCD30_I2C_ADDR, &value, 3, 1000);
//	size=sprintf( (char *)Data, "Value 0 %x\r\n", value[0]);
//	HAL_UART_Transmit(&huart2, Data, size, 1000);
//	size=sprintf( (char *)Data, "Value 1 %x\r\n", value[1]);
//	HAL_UART_Transmit(&huart2, Data, size, 1000);
//	if ((value[0]<<8 | value[1]) == 834){
//		return STATUS_OK;
//	} else{
//		return STATUS_FAIL;
//	}
//}
//
//void SCD30_Start(){
//	uint8_t cmd[5]={START_MEASURE_MSB, START_MEASURE_LSB, PRESSURE_MSB, PRESSURE_LSB, CRC};
//	HAL_I2C_Master_Transmit(scd30_i2c, SCD30_I2C_ADDR, cmd, 5, 1000);
//	WriteMsg("Measurements started \r\n");
//}
//
//Status_Flag SCD30_Read(float *temperature3, float* humidity3, float* CO2){
//	uint8_t value[18]={0};
//	unsigned char buffer[4];
//	unsigned int tempU32;
//	float result=0;
//
//	uint8_t cmd[2]={READ_MSB, READ_LSB};
//	HAL_I2C_Master_Transmit(scd30_i2c, SCD30_I2C_ADDR, cmd, 2, 1000);
//	HAL_I2C_Master_Receive(scd30_i2c, SCD30_I2C_ADDR, &value, 18, 1000);
//
//	tempU32 = (unsigned int) ((((unsigned int)value[0]) << 24) | (((unsigned int)value[1]) << 16) |
//								 (((unsigned int)value[3]) << 8) |
//								 ((unsigned int)value[4]));
//	result= *(float*)&tempU32;
//	*CO2= result;
//	tempU32=0;
//
//	tempU32 = (unsigned int) ((((unsigned int)value[6]) << 24) | (((unsigned int)value[7]) << 16) |
//								 (((unsigned int)value[9]) << 8) |
//								 ((unsigned int)value[10]));
//	result= *(float*)&tempU32;
//	*temperature3= result;
//	tempU32=0;
//
//	tempU32 = (unsigned int) ((((unsigned int)value[12]) << 24) | (((unsigned int)value[13]) << 16) |
//								 (((unsigned int)value[15]) << 8) |
//								 ((unsigned int)value[16]));
//	result= *(float*)&tempU32;
//	*humidity3= result;
//	tempU32=0;
//
//
//
//
////	buffer[0]=value[6];
////	buffer[1]=value[7];
////	buffer[2]=value[9];
////	buffer[3]=value[10];
////	result= *(float*)&tempU32;
//	return STATUS_OK;
//}
//
//
