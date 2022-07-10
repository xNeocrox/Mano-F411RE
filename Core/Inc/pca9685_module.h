/*
 * pca9685_module.h
 *
 *  Created on: 20 mar. 2022
 *      Author: ismae
 */

#ifndef INC_PCA9685_MODULE_H_
#define INC_PCA9685_MODULE_H_

#include "stm32f4xx_hal.h"

void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address);
void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off);
void pca9685_Degrees2PWM(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint8_t grados);

void Indice_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Corazon_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Anular_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Menique_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Pulgar_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void EjeY_Mov(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void EjeX_Mov(TIM_HandleTypeDef htim, uint8_t grados);

#endif /* INC_PCA9685_MODULE_H_ */
