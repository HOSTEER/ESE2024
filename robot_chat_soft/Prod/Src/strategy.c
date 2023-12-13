#include "strategy.h"



#define ZONE_MILIEU 2
#define ZONE_DROITE 3
#define ZONE_GAUCHE 1
#define ZONE_HAUTE 1
#define ZONE_BASSE 3

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


int strategy(strat_mode_t * strat_mode, hOdometry_t * hOdometry){

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

int champ_vectoriel(strat_mode_t * strat_mode, hOdometry_t * hOdometry){
	uint8_t zone = zone_sorting(&champ_vect, hOdometry);

	switch(zone){
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	case 9:
		break;
	}
	return 0;
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

int32_t zone_lineaire(uint8_t zone){
	int8_t axe, sens;
	int32_t dir_vect[2];
	if(zone == 2 || zone == 8){
		axe = 1; //Axe horizontal
		if(zone == 2)
			sens = 1;
		else
			sens = -1;
	}
	else{
		axe = 2; //Axe vertical
		if(zone == 4)
			sens = 1;
		else
			sens = -1;
	}

	if(axe == 1){
		dir_vect[0] = hOdometry.y-(champ_vect.)
	}
}

int32_t zone_circulaire(uint8_t zone){

}
