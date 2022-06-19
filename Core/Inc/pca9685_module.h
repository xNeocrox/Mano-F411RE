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

void Indice(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Corazon(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Anular(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Menique(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Pulgar(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);

#endif /* INC_PCA9685_MODULE_H_ */
