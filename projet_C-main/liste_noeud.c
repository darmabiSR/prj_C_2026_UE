#define _GNU_SOURCE
#include "liste_noeud.h"
#include <stdlib.h>
#include <math.h>
#include "coord.h"
#include <stdbool.h>

typedef struct cellule_t {
    float cout;
    coord_t noeud;
    coord_t precedent;
    struct cellule_t* suiv;
} cellule_t;

struct liste_noeud_t {
    cellule_t* debut;
};

static coord_t coord_abscente(void){
    return creer_coord(-1,-1);
}

liste_noeud_t creer_liste(void){
    liste_noeud_t liste = malloc(sizeof(*liste));
    (*liste).debut = NULL;
    return liste;
}

void detruire_liste(liste_noeud_t* liste_ptr){
    liste_noeud_t liste = *liste_ptr;
    cellule_t* pointeur = (*liste).debut;
    while (pointeur != NULL) {
        cellule_t* suivant = (*pointeur).suiv;
        free(pointeur);
        pointeur = suivant;
    }
    free(liste);
    *liste_ptr = NULL;
}

bool est_vide_liste(liste_noeud_t liste){
    return ((*liste).debut == NULL);
}

bool contient_noeud_liste(liste_noeud_t liste, coord_t noeud){
    cellule_t* pointeur;
    for (pointeur = (*liste).debut; pointeur != NULL; pointeur = (*pointeur).suiv){
        if (memes_coord(noeud, (*pointeur).noeud)){
            return true;
        }
    } return false;
}

bool contient_arrete_liste(liste_noeud_t liste, coord_t source, coord_t destination){
    bool test = false;
    cellule_t* pointeur = (*liste).debut;
    while (test==false && pointeur != NULL){
        if (memes_coord((*pointeur).noeud, destination) && memes_coord((*pointeur).precedent, source)){
            test=true;
        }
    }
    return test;
}

float cout_noeud_liste(liste_noeud_t liste, coord_t noeud){
    cellule_t* pointeur = (*liste).debut;
    
    while (memes_coord(noeud, (*pointeur).noeud) == false && pointeur != NULL){
        pointeur = pointeur->suiv;
    }
    if (memes_coord(noeud, (*pointeur).noeud) == true) {
        return pointeur->cout;
    } else {
        return INFINITY;
    }
}

coord_t precedent_noeud_liste(liste_noeud_t liste, coord_t noeud){
    cellule_t* pointeur = (*liste).debut;
    
    while (memes_coord(noeud, (*pointeur).noeud) == false && pointeur != NULL){
        pointeur = pointeur->suiv;
    }
    if (memes_coord(noeud, (*pointeur).noeud) == true) {
        return (*pointeur).precedent;
    } else {
        return coord_abscente();
    }


}

coord_t min_noeud_liste(liste_noeud_t liste){
    cellule_t* mini = liste->debut;
    for (cellule_t* pointeur = mini->suiv; pointeur != NULL; pointeur = pointeur->suiv){
        if ((*pointeur).cout < mini->cout){
            mini = pointeur;
        }

    }
    return (*mini).noeud;


}

void inserer_noeud_liste(liste_noeud_t liste, coord_t noeud, coord_t precedent, float cout){
    if (contient_noeud_liste(liste, noeud)){
        for (cellule_t* pointeur = liste->debut; pointeur != NULL; pointeur = pointeur->suiv){
            if (memes_coord(noeud, (*pointeur).noeud)){
                (*pointeur).precedent = precedent;
                pointeur->cout = cout;
            }  
        }

    } else{
        cellule_t* cellule = malloc(sizeof(*cellule));
        (*cellule).noeud = noeud;
        (*cellule).precedent= precedent;
        (*cellule).cout = cout;
        (*cellule).suiv = (*liste).debut;
        (*liste).debut = cellule;
    }
}

void supprimer_noeud_liste(liste_noeud_t liste, coord_t noeud){
    bool test = false;
    if (contient_noeud_liste(liste, noeud)){
        cellule_t* pointeur = (*liste).debut;
        cellule_t* prec = NULL;
        while (test==false && pointeur != NULL){
            if (memes_coord((*pointeur).noeud, noeud)){
                if (prec==NULL){
                    (*liste).debut = (*pointeur).suiv;
                } else {
                    (*prec).suiv = (*pointeur).suiv;
                }
                free(pointeur);
                test=true;
            }
            prec = pointeur;
            pointeur = pointeur->suiv;
        } 
    }
}