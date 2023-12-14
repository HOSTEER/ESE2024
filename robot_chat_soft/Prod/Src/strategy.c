#include "strategy.h"



#define ZONE_MILIEU 2
#define ZONE_DROITE 3
#define ZONE_GAUCHE 1
#define ZONE_HAUTE 1
#define ZONE_BASSE 3

#define ERR_ZONE_404 1

/* 			|				|
 * 		1	|		2		|	3
 * ----------------------------------
 * 			|				|
 * 		4	|		5		|	6
 * 			|				|
 * ----------------------------------
 * 		7	|		8		|	9
 * 			|				|
 */


champ_vect_t champ_vect;


void init_champ_vect(void){
	champ_vect.vitesse = 10;
	champ_vect.pt_lim_haut = (500<<16);
	champ_vect.pt_lim_bas = -1*champ_vect.pt_lim_haut;
	champ_vect.pt_lim_droite = (1000<<16);
	champ_vect.pt_lim_gauche = -1*champ_vect.pt_lim_droite;

	champ_vect.offset_gauche = -1*(10<<16);
	champ_vect.offset_droite = -1*champ_vect.offset_gauche;
	champ_vect.offset_haut = (10<<16);
	champ_vect.offset_bas = -1*champ_vect.offset_haut;
}


int8_t strategy(strat_mode_t * strat_mode, hOdometry_t * hOdometry){
	int32_t dir_vect[2];
	if((*strat_mode & 0xF000) == HUNTER){
		//Le robot est chasseur
		if((*strat_mode & 0xFF) != NO_OBSTACLE){
			//Obstacle détecté

		}
		else{
			//Aucun obstacle

		}
	}
	else{
		//Le robot est la proie
		if((*strat_mode & 0xFF) != NO_OBSTACLE){
			//Obstacle détecté

		}
		else{
			//Aucun obstacle


		}
	}
	return 0;
}

int8_t champ_vectoriel(strat_mode_t * strat_mode, hOdometry_t * hOdometry, int32_t * dir_vect){
	uint8_t zone = zone_sorting(&champ_vect, hOdometry);
	int8_t status;

	if(zone%2 == 0){
		status = zone_lineaire(&champ_vect, hOdometry, zone, dir_vect);
	}
	else{
		status += zone_circulaire(&champ_vect, hOdometry, zone, dir_vect);
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

	if((zone_h + 3*(zone_h - 1)) == 5){
		//Cas de la zone milieu, il faut calculer la position le long des droites
		int32_t fAB = hOdometry->x*(champ_vect->pt_lim_bas - champ_vect->pt_lim_haut)/(champ_vect->pt_lim_droite - champ_vect->pt_lim_gauche);
		int32_t fCD = hOdometry->x*(champ_vect->pt_lim_haut - champ_vect->pt_lim_bas)/(champ_vect->pt_lim_droie - champ_vect->pt_lim_gauche);

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
		return zone_h + 3*(zone_h - 1);
	}

}

int8_t zone_lineaire(champ_vect_t * champ_vect, hOdometry_t * hOdometry, uint8_t zone, int32_t * dir_vect){
	int8_t axe, sens;
	//int32_t dir_vect[2];
	switch(zone){
	case 2: //Zone haute
		sens = +1;
		dir_vect[1] = ((champ_vect->pt_lim_haut + champ_vect->offset_haut) - hOdometry->y)*champ_vect->vitesse_correctrice;
		dir_vect[0] = sens*champ_vect->vitesse_avance;
		break;
	case 4: //Zone Gauche
		sens = +1;
		dir_vect[0] = ((champ_vect->pt_lim_gauche + champ_vect->offset_gauche) - hOdometry->x)*champ_vect->vitesse_correctrice;
		dir_vect[1] = sens*champ_vect->vitesse_avance;
		break;
	case 6: //Zone droite
		sens = -1;
		dir_vect[0] = ((champ_vect->pt_lim_droite + champ_vect->offset_droite) - hOdometry->x)*champ_vect->vitesse_correctrice;
		dir_vect[1] = sens*champ_vect->vitesse_avance;
		break;
	case 8: //Zone basse
		sens = -1;
		dir_vect[1] = ((champ_vect->pt_lim_bas + champ_vect->offset_bas) - hOdometry->y)*champ_vect->vitesse_correctrice;
		dir_vect[0] = sens*champ_vect->vitesse_avance;
		break;
	default:
		return ERR_ZONE_404;
	}
	return 0;
}

int8_t zone_circulaire(champ_vect_t * champ_vect, hOdometry_t * hOdometry, uint8_t zone, int32_t * dir_vect){
	//int32_t dir_vect[2] = {0};
	int32_t xb, yb;
	int32_t Dx, Dy;
	int32_t d, norm;

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
	//TODO norm = sqrt(Dx*Dx + Dy*Dy);
	//TODO d = l - norm

	//Calcul consigne de direction
	//vect colinéaire = {-Dx/norm, -Dy/norm}
	//vect tangentiel = {Dy/norm, -Dx/norm}
	//consigne = d*vect_colin + sens*vitesse*vect_tang
	dir_vect[0] = (-d*Dx)/norm + (champ_vect->vitesse_avance*Dy)/norm;
	dir_vect[1] = (-d*Dy)/norm + (-champ_vect->vitesse_avance*Dx)/norm;

	return 0;
}
