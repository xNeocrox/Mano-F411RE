/*
 * app_common_typedef.h
 *
 *  Created on: 20 mar. 2022
 *      Author: ismae
 */

#ifndef INC_APP_COMMON_TYPEDEF_H_
#define INC_APP_COMMON_TYPEDEF_H_

struct datos
{
	uint8_t Move_pulgar;
	uint8_t Move_indice;
	uint8_t Move_corazon;
	uint8_t Move_anular;
	uint8_t Move_menique;
	uint8_t Move_ejeX;
	uint8_t Move_ejeY;
};

struct presion
{
	uint8_t Pres_Pulgar;
	uint8_t Pres_Indice;
	uint8_t Pres_Corazon;
	uint8_t Pres_Anular;
	uint8_t Pres_Menique;
};

typedef enum
{
	OPEN_HAND,
	RUNNING_CODE
}Init_State;

#endif /* INC_APP_COMMON_TYPEDEF_H_ */
