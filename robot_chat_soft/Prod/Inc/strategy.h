/*
 * strategy.h
 *
 *  Created on: Dec 13, 2023
 *      Author: les_victor
 */

#ifndef INC_STRATEGY_H_
#define INC_STRATEGY_H_

#include <stdint.h>
#include "odometry.h"

typedef enum{
	HUNTER			= 0x1000,
	PREY			= 0x2000,
	NO_OBSTACLE		= 0x0000,
	FALL_FORWARD 	= 0x0001,
	FALL_BACKWARD 	= 0x0002,
	COLLIDE 		= 0x0010,
	TURN_TRIGO		= 0x0100,
	TURN_CLOCK		= 0x0200
}strat_mode_t;

typedef struct CHAMP_VECT_STRUCT_T{
	int16_t vitesse;
	int32_t pt_lim_gauche, pt_lim_droite;
	int32_t pt_lim_haut,  pt_lim_bas;
	int32_t offset_gauche, offset_droite, offset_haut, offset_bas;
}champ_vect_t;


int strategy(strat_mode_t * strat_mode, hOdometry_t * hOdometry);

#endif /* INC_STRATEGY_H_ */
