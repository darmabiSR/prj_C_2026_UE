#define _GNU_SOURCE
#include "grille.h"
#include <stdlib.h>
#include <math.h>
#include <assert.h>

/**
 * coord_lin : fonction statique (de module) qui transforme des coordonnées
 * cartésiennes dans la grille en coordonnées "linéaires" permettant d'adresser
 * le tableau contenu dans la grille.
 *
 * @param grille grille avec le bon système de coordonnées
 * @param position position dans la grille en coordonnées cartésiennes entières
 * @return position dans le tableau en coordonnée linéaire
 */
static size_t coord_lin(grille_t grille, coord_t position) {
    return (size_t)get_y(position)*(size_t)(*grille).largeur+(size_t)get_x(position);
}

grille_t creer_grille(int largeur, int profondeur){
    grille_t g = malloc(sizeof(*g));
    (*g).largeur = largeur;
    (*g).profondeur = profondeur;
    size_t n = (size_t)largeur * (size_t)profondeur;
    (*g).hauteurs= calloc(n, sizeof(float));
    return g;
}

void detruire_grille(grille_t grille){
    free((*grille).hauteurs);
    (*grille).hauteurs = NULL;
    free(grille);
}

bool dans_les_bornes(grille_t grille, coord_t position){
    int x = get_x(position);
    int y = get_y(position);
    return (0<= x && x< (*grille).largeur && 0<=y && y< (*grille).profondeur);
}

int get_largeur(grille_t grille){
    assert(grille != NULL);
    return (*grille).largeur;
}

int get_profondeur(grille_t grille){
    assert (grille != NULL);
    return (*grille).profondeur;


}

coord_t inferieur_gauche(grille_t grille){
    assert (grille != NULL);
    return creer_coord(0,0);

}

coord_t superieur_droit(grille_t grille){
    assert (grille != NULL);
    return creer_coord((*grille).largeur-1,(*grille).profondeur-1);

}

void set_hauteur(grille_t grille, coord_t position, float hauteur){
    assert (grille != NULL);
    assert(dans_les_bornes(grille,position));
    (*grille).hauteurs[coord_lin(grille, position)]= hauteur;

}

float get_hauteur(grille_t grille, coord_t position){
    assert (grille != NULL);
    assert(dans_les_bornes(grille,position));
    return (*grille).hauteurs[coord_lin(grille, position)];
    

}

size_t get_voisins(grille_t grille, coord_t position, float seuil, coord_t** voisins){
    assert (grille != NULL);

}