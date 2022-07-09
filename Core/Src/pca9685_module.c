/*
 * pca9685_module.c
 *
 *  Created on: 20 mar. 2022
 *      Author: ismae
 */

#include "pca9685_module.h"



void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address)
{
	#define PCA9685_MODE1 0x00

	uint8_t initStruct[2];
	uint8_t prescale = 3;
	HAL_I2C_Master_Transmit(hi2c, address, PCA9685_MODE1,1 ,1);
	uint8_t oldmode = 0;

	uint8_t newmode = ((oldmode & 0x7F)|0x10);
	initStruct[0] = PCA9685_MODE1;
	initStruct[1] = newmode;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	initStruct[1] = prescale;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	initStruct[1] = oldmode;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	osDelay(5);
	initStruct[1] = (oldmode|0xA1);
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
}

void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off)
{
	uint8_t outputBuffer[5] = {0x06 + 4*num,on,(on >> 8), off, (off >> 8)};
	HAL_I2C_Master_Transmit(hi2c, address, outputBuffer, 5, 1);
}

void pca9685_Degrees2PWM(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint8_t grados)
{
	uint16_t off;
	off = (9.45*grados) + 300;

	if(off > 2000){
		off = 2000;
	}
	else if(off < 300){
		off = 300;
	}

	pca9685_pwm(hi2c, address, num, 0, off);

}

void Indice_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 15, grados);
	pca9685_Degrees2PWM(hi2c, address, 11, grados);
	pca9685_Degrees2PWM(hi2c, address, 7, grados);
}

void Corazon_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 14, grados);
	pca9685_Degrees2PWM(hi2c, address, 10, grados);
	pca9685_Degrees2PWM(hi2c, address, 6, grados);
}

void Anular_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 13, grados);
	pca9685_Degrees2PWM(hi2c, address, 9, grados);
	pca9685_Degrees2PWM(hi2c, address, 5, grados);
}

void Menique_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 12, grados);
	pca9685_Degrees2PWM(hi2c, address, 8, grados);
	pca9685_Degrees2PWM(hi2c, address, 4, grados);
}

void Pulgar_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	uint8_t grados_t;
	grados_t = ((180-grados)/1.8)+50;
	pca9685_Degrees2PWM(hi2c, address, 1, grados_t);
	pca9685_Degrees2PWM(hi2c, address, 2, grados);
	pca9685_Degrees2PWM(hi2c, address, 3, grados);
}
