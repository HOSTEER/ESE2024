#include "strategy.h"

#include "config.h"


#define ZONE_MILIEU 2
#define ZONE_DROITE 3
#define ZONE_GAUCHE 1
#define ZONE_HAUTE 1
#define ZONE_BASSE 3

#define ERR_ZONE_404 1

/*	Schéma de répartition des zones du champ vectoriel
 *  		|				|
 * 		1	|		2		|	3
 * ----------------------------------
 * 			|		^y		|
 * 			|	5	| 		|
 * 		4	|		.->x	|	6
 * 			|				|
 * ----------------------------------
 * 		7	|		8		|	9
 * 			|				|
 */



static champ_vect_t champ_vect;


void init_champ_vect(void){
	champ_vect.vitesse_avance = (VITESSE_CIRCUIT<<16);
	champ_vect.vitesse_correctrice = (VITESSE_CORRECTION<<16);
	champ_vect.pt_lim_haut = (HAUTEUR_CHAMP<<16);
	champ_vect.pt_lim_bas = -1*champ_vect.pt_lim_haut;
	champ_vect.pt_lim_droite = (LARGEUR_CHAMP<<16);
	champ_vect.pt_lim_gauche = -1*champ_vect.pt_lim_droite;

	champ_vect.offset = (CHAMP_VECT_MARGE<<16);
}


int8_t strategy(strat_mode_t * strat_mode, hOdometry_t * hOdometry){
	vector_t dir_vect;
	if((*strat_mode & 0xF000) == HUNTER){
		//Le robot est chasseur
		if((*strat_mode & 0xFF) == NO_OBSTACLE){
			//Aucun obstacle
			//TODO appel fonction detection obstacle le + proche
			//TODO somme 2 consignes
			//TODO màj consigne commande
		}
		else{
			//Obstacle détecté
			printf("Comportement obstacle \n\r");
			switch(*strat_mode&0xFF){
			case FALL_FORWARD:
				//TODO commande recul rectiligne
				break;
			case FALL_BACKWARD:
				//TODO commande avance rectiligne
				break;
			case FALL_FORWARD | COLLIDE:
				*strat_mode = (*strat_mode&0xFFF) | PREY;
				//TODO commande recul rectiligne
				break;
			case FALL_BACKWARD | COLLIDE:
				*strat_mode = (*strat_mode&0xFFF) | PREY;
				//TODO commande avance rectiligne
				break;
			default: //COLLIDE sans FALL_x
				*strat_mode = (*strat_mode&0xFFF) | PREY;
				//TODO consigne vitesse nulle
				break;
			}
		}
	}
	else{
		//Le robot est la proie
		if((*strat_mode & 0xFF) == NO_OBSTACLE){
			//Aucun obstacle
			champ_vectoriel(&champ_vect, strat_mode, hOdometry, &dir_vect);
			//TODO appel fonction detection obstacle le + proche
			//TODO somme 2 consignes
			//TODO màj consigne commande
			printf("Comportement fuite \n\r");
		}
		else{
			//Obstacle détecté
			printf("Comportement obstacle \n\r");
			switch(*strat_mode&0xFF){
			case FALL_FORWARD:
				//TODO commande recul rectiligne
				break;
			case FALL_BACKWARD:
				//TODO commande avance rectiligne
				break;
			case FALL_FORWARD | COLLIDE:
				*strat_mode = (*strat_mode&0xFFF) | HUNTER;
				//TODO commande recul rectiligne
				break;
			case FALL_BACKWARD | COLLIDE:
				*strat_mode = (*strat_mode&0xFFF) | HUNTER;
				//TODO commande avance rectiligne
				break;
			default: //COLLIDE sans FALL_x
				*strat_mode = (*strat_mode&0xFFF) | HUNTER;
				//TODO consigne vitesse nulle
				break;
			}
		}
	}
	printf("x: %d, y: %d\n\r", dir_vect.x>>16, dir_vect.y>>16);
	return 0;
}

int8_t champ_vectoriel(champ_vect_t * champ_vect, strat_mode_t * strat_mode, hOdometry_t * hOdometry, vector_t * dir_vect){
	uint8_t zone = zone_sorting(champ_vect, hOdometry);
	printf("La zone du robot est %d\n\r", zone);
	int8_t status;

	if(zone%2 == 0){
		status = zone_lineaire(champ_vect, strat_mode, hOdometry, zone, dir_vect);
	}
	else{
		status = zone_circulaire(champ_vect, strat_mode, hOdometry, zone, dir_vect);
	}
	return status;
}

int8_t zone_sorting(champ_vect_t * champ_vect, hOdometry_t * hOdometry){
	int8_t zone_v; //Zone verticale dans laquelle est le robot
	int8_t zone_h; //Zone horizontale dans laquelle est le robot

	//Detection de la zone parmi les 9 dans laquelle se trouve le robot
	if (hOdometry->x >= champ_vect->pt_lim_droite){
		zone_h = ZONE_DROITE;
	}
	else if (hOdometry->x < champ_vect->pt_lim_gauche){
		zone_h = ZONE_GAUCHE;
	}
	else{
		zone_h = ZONE_MILIEU;
	}

	if (hOdometry->y >= champ_vect->pt_lim_haut){
		zone_v = ZONE_HAUTE;
	}
	else if (hOdometry->y < champ_vect->pt_lim_bas){
		zone_v = ZONE_BASSE;
	}
	else{
		zone_v = ZONE_MILIEU;
	}

	//Cas de la zone milieu, il faut calculer la position le long des droites
	if((zone_h + 3*(zone_v - 1)) == 5){

		int32_t fAB = fixed_mul_16(hOdometry->x, fixed_div_16(champ_vect->pt_lim_bas - champ_vect->pt_lim_haut, champ_vect->pt_lim_droite - champ_vect->pt_lim_gauche));
		int32_t fCD = fixed_mul_16(hOdometry->x, fixed_div_16(champ_vect->pt_lim_haut - champ_vect->pt_lim_bas, champ_vect->pt_lim_droite - champ_vect->pt_lim_gauche));

		if(hOdometry->y >= fAB){
			if(hOdometry->y >= fCD){
				//Zone haute
				return 2;
			}
			else{
				//Zone droite
				return 6;
			}
		}
		else{
			if(hOdometry->y >= fCD){
				//Zone gauche
				return 4;
			}
			else{
				//Zone basse
				return 8;
			}
		}
	}
	else{
		return zone_h + 3*(zone_v - 1);
	}

}

int8_t zone_lineaire(champ_vect_t * champ_vect, strat_mode_t * strat_mode, hOdometry_t * hOdometry, uint8_t zone, vector_t * dir_vect){
	int32_t sens;

	if((*strat_mode & 0x0F00)  == TURN_TRIGO){
		sens = +1;
	}
	else{
		sens = -1;
	}

	//int32_t dir_vect[2];
	switch(zone){
	case 2: //Zone haute
		//sens = +1;
		dir_vect->y = fixed_mul_16(((champ_vect->pt_lim_haut + champ_vect->offset) - hOdometry->y), champ_vect->vitesse_correctrice);
		dir_vect->x = +sens*champ_vect->vitesse_avance;
		break;
	case 4: //Zone Gauche
		//sens = +1;
		dir_vect->x = fixed_mul_16(((champ_vect->pt_lim_gauche - champ_vect->offset) - hOdometry->x), champ_vect->vitesse_correctrice);
		dir_vect->y = +sens*champ_vect->vitesse_avance;
		break;
	case 6: //Zone droite
		//sens = -1;
		dir_vect->x = fixed_mul_16(((champ_vect->pt_lim_droite + champ_vect->offset) - hOdometry->x), champ_vect->vitesse_correctrice);
		dir_vect->y = -sens*champ_vect->vitesse_avance;
		break;
	case 8: //Zone basse
		//sens = -1;
		dir_vect->y = fixed_mul_16(((champ_vect->pt_lim_bas - champ_vect->offset) - hOdometry->y), champ_vect->vitesse_correctrice);
		dir_vect->x = -sens*champ_vect->vitesse_avance;
		break;
	default:
		return ERR_ZONE_404;
	}
	return 0;
}

int8_t zone_circulaire(champ_vect_t * champ_vect, strat_mode_t * strat_mode, hOdometry_t * hOdometry, uint8_t zone, vector_t * dir_vect){
	//int32_t dir_vect[2] = {0};
	int32_t xb, yb;
	int32_t Dx, Dy;
	int32_t d, norm;

	int32_t sens;

	if((*strat_mode & 0x0F00)  == TURN_TRIGO){
		sens = +1;
	}
	else{
		sens = -1;
	}

	switch(zone){
	case 1:
		xb = champ_vect->pt_lim_gauche;
		yb = champ_vect->pt_lim_haut;
		break;
	case 3:
		xb = champ_vect->pt_lim_droite;
		yb = champ_vect->pt_lim_haut;
		break;
	case 7:
		xb = champ_vect->pt_lim_gauche;
		yb = champ_vect->pt_lim_bas;
		break;
	case 9:
		xb = champ_vect->pt_lim_droite;
		yb = champ_vect->pt_lim_bas;
		break;
	default:
		return ERR_ZONE_404;
	}

	Dx = (hOdometry->x - xb);
	Dy = (hOdometry->y - yb);
	vector_t Dvect = {Dx, Dy};
	CORDIC_vector(&Dvect);
	norm = Dvect.norm;

	d = champ_vect->offset - norm;
	//Calcul consigne de direction
	//vect colinéaire = {-Dx/norm, -Dy/norm}
	//vect tangentiel = {Dy/norm, -Dx/norm}
	//consigne = d*vect_colin + sens*vitesse*vect_tang
	dir_vect->x = fixed_div_16(fixed_mul_16(-d, Dx), norm) + sens*fixed_div_16(fixed_mul_16(champ_vect->vitesse_avance, Dy), norm);
	dir_vect->y = fixed_div_16(fixed_mul_16(-d, Dy), norm) + sens*fixed_div_16(fixed_mul_16(-champ_vect->vitesse_avance, Dx), norm);

	return 0;
}
