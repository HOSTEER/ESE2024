#define length
int grille[16*16];

int cases_left;
void main(void){
    int x, y ={0};
    while(1){
        if(grille[x + length*y] == -1){
            for()
        }
    }
}

int ligne_valide(int n){

    for(int i =0;i<16;i++){
        if(!dans_ligne(n, i))
        return 1;
    }
    return 0;
}

int dans_ligne(int n, int nbr){
    for(int i =0;i<length;i++){
        if(grille[length*n + i] == nbr)
        return 0;
    }
    return 1;
}

int colonne_valide(int n){

    for(int i =0;i<16;i++){
        if(!dans_colonne(n, i))
        return 1;
    }
    return 0;
}

int dans_colonne(int n, int nbr){
    for(int i =0;i<length;i++){
        if(grille[length*i + n] == nbr)
        return 0;
    }
    return 1;
}

int carre_valide(int ligne, int colonne){
    int y_ref = ligne/(ligne%4);
    int x_ref = colonne/(colonne%4);
    for(int i =0;i<16;i++){
        if(!dans_carre(x_ref, y_ref, i))
        return 1;
    }
    return 0;

}

int dans_carre(int x_ref, int y_ref, int nbr){
    for(int j=0;j<4;j++){
        for(int i =0;i<4;i++){
            if(grille[length*(y_ref+j) + x_ref + i] == nbr)
            return 0;
        }
    }
    return 1;
}