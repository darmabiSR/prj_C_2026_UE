#ifndef GRID_H
#define GRID_H

#include <stdbool.h>
#include <stddef.h>
#include "coord.h"

/**
 * Type grille_t représentant une heightmap, caractérisé par
 *  - une largeur (entière)
 *  - une profondeur (entière)
 *  - un tableau de flottants contenant les données de hauteur
 */
struct grille_t {
    int largeur;
    int profondeur;
    float* hauteurs;
};

typedef struct grille_t* grille_t;

/**
 * creer_grille : crée une grille de largeur et de profondeur données.
 * Initialement la grille ne contient que des 0.0.
 *
 * Pré-conditions : largeur > 0, profondeur > 0
 * Post-conditions : pour tout i,j, 0 <= i < largeur, 0 <= j < profondeur,
 *          get_hauteur(creer_grille(largeur, profondeur), i, j) == 0.0
 *
 * @param largeur largeur de la grille (taille selon l'axe des X)
 * @param profondeur profondeur de la grille (taille selon l'axe des Y)
 * @return grille nouvellement créée
 */
grille_t creer_grille(int largeur, int profondeur);

/**
 * detruire_grille : libère la mémoire utilisée par une grille.
 *
 * Pré-conditions : grille créée avec creer_grille, non déjà libérée
 * Post-conditions : mémoire libérée
 *
 * @param grille grille à libérer
 */
void detruire_grille(grille_t grille);


/**
 * dans_les_bornes : test si des coordonnées sont bien dans les bornes
 * de la grille.
 *
 * Pré-conditions : grille créée avec creer_grille
 * Post-conditions : 
 *
 * @param grille grille support du test
 * @param position position à tester
 * @return true ssi position est dans les bornes de grilles
 */
bool dans_les_bornes(grille_t grille, coord_t position);

/**
 * get_largeur : récupère la largeur de la grille.
 *
 * Pré-conditions : grille créée avec creer_grille
 * Post-conditions : 
 *
 * @param grille grille dont on veut la largeur
 * @return largeur de la grille
 */
int get_largeur(grille_t grille);

/**
 * get_profondeur : récupère la profondeur de la grille.
 *
 * Pré-conditions : grille créée avec creer_grille
 * Post-condition :
 *
 * @param grille dont on veut la profondeur
 * @return profondeur de la grille
 */
int get_profondeur(grille_t grille);


/**
 * inferieur_gauche : coordonnées du point inférieur gauche de la grille
 * (dans un repère orienté en sens direct, c'est-à-dire de gauche à droite et 
 * de bas en haut).
 *
 * Pré-conditions : grille créée avec creer_grille
 * Post-conditions : si c = inferieur_gauche(g) alors dans_les_bornes(g, c)
 *
 * @param grille grille dont on veut le coin inférieur gauche
 * @return coordonnées du point demandé
 */
coord_t inferieur_gauche(grille_t grille);

/**
 * superieur_droit : coordonnées du point supérieur droit de la grille
 * (dans un repère orienté en sens direct, c'est-à-dire de gauche à droite et 
 * de bas en haut).
 *
 * Pré-conditions : grille créée avec creer_grille
 * Post-conditions : si c = superieur_droit(g) alors dans_les_bornes(g, c)
 *
 * @param grille grille dont on veut le coin supérieur droit
 * @return coordonnées du point demandé
 */
coord_t superieur_droit(grille_t grille);

/**
 * set_hauteur : modifie la hauteur dans la grille au coordonnées demandées.
 * Le reste de la grille est inchangé.
 *
 * Pré-conditions : grille créée avec creer_grille, dans_les_bornes(grille, position)
 * Post-conditions : get_hauteur(grille', position) = hauteur
 *                   pour tout c /+ position, get_hauteur(grille', c) = get_hauteur(grille, c)
 *
 * @param grille grille dont on veut modifier une hauteur
 * @param position position à modifier
 * @param hauteur nouvelle hauteur à mettre dans la grille
 */
void set_hauteur(grille_t grille, coord_t position, float hauteur);

/**
 * get_hauteur : récupère la hauteur dans la grille aux coordonnées demandées.
 *
 * Pré-conditions : grille créée avec creer_grille, dans_les_bornes(grille, position)
 * Post-conditions : 
 *
 * @param grille grille dont on veut la hauteur
 * @param position position de la hauteur à récupérer
 * @return hauteur dans la grille au point de coordonnées position
 */
float get_hauteur(grille_t grille, coord_t position);

/**
 * get_voisins : récupère les voisins directs (orthogonnaux) de la coordonnée donnée, 
 * tels que le dénivelé  entre la hauteur au point donné et lesdits voisins soit 
 * inférieure au seuil donné.
 *
 * La fonction se charge d'allouer de la mémoire pour stocker les voisins dans voisins,
 * et renvoie la taille du tableau ainsi alloué. C'est à l'utilisateur de libérer la mémoire.
 *
 * Pré-conditions : grille créée avec creer_grille, voisins non NULL, seuil >= 0.0
 * Post-conditions :
 *   - n = get_voisins(..., voisins) => voisins contient au moins n cases, 0 <= n <= 4
 *   - pour tout c dans voisins, dans_les_bornes(grille, c)
 *   - pour tout c dans voisins, |get_hauteur(grille, position) - get_hauteur(grille, c)| <= seuil
 *
 * @param grille grille dont on veut extraire des voisins
 * @param position position du noeud dont on veut les voisins
 * @param seuil seuil au-dessus duquel on rejète le voisin candidat
 * @param voisins [out] tableau récueillant les voisins de position dans grille
 * @return taille du tableau voisins alloué par la fonction
 */
size_t get_voisins(grille_t grille, coord_t position, float seuil, coord_t** voisins);

#endif

