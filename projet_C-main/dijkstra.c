#include "dijkstra.h"
#include <math.h>
//#include "animation.h"

/**
 * cout : calcule le coût pour rejoindre le noeud suivant depuis le noeud
 * courant. Ce coût est égal à la distance euclidienne entre les deux noeuds,
 * plus le dénivelé entre ces deux noeuds s'il est positif.
 * 
 * Pré-conditions :
 *  - dans_les_bornes(grille, courant)
 *  - dans_les_bornes(grille, suivant)
 * Post-conditions :
 *  - cout > 0, potentiellement infini (INFINITIY)
 *
 * @param grille heightmap contenant les hauteurs
 * @param courant noeud duquel on part pour calculer le coût
 * @param suivant noeud auquel on arrive pour calculer le coût
 * @return coût pour passer de courant à suivant
 */
static float cout(grille_t grille, coord_t courant, coord_t suivant) {
    float x = courant.x - suivant.x;
    float y = courant.y - suivant.y;
    float distance = sqrt(x*x + y*y);
    float denivele = get_hauteur(grille, suivant) - get_hauteur(grille, courant);
    if (denivele < 0) denivele = 0;
    return denivele + distance;
}

/**
 * construire_chemin_vers - Construit le chemin depuis le noeud de départ donné vers le
 * noeud donné. On passe un chemin en entrée-sortie de la fonction, qui est mis à jour
 * par celle-ci.
 *
 * Ce sous-programme fonctionne récursivement :
 *  1. Si le noeud est égal au noeud de départ, on a fini
 *  2. Sinon, on construit le chemin du départ au noeud précédent (appel récursif)
 *  3. Dans tous les cas, on ajoute le noeud au chemin, avec les caractéristiques associées dans visites
 *
 * @param chemin [in/out] chemin dans lequel enregistrer les étapes depuis le départ vers noeud
 * @param visites [in] liste des noeuds visités créée par l'algorithme de Dijkstra
 * @param source noeud de départ du chemin
 * @param noeud noeud vers lequel on veut construire le chemin depuis le départ
 */
void construire_chemin_vers(liste_noeud_t** chemin, liste_noeud_t* visites, coord_t source, coord_t noeud){
    if (noeud.x != source.x || noeud.y != source.y){
        coord_t prev = precedent_noeud_liste(visites, noeud);
        inserer_noeud_liste(*chemin, noeud, prev, cout_noeud_liste(visites, noeud));
        construire_chemin_vers(chemin, visites, source, prev);
    }
}

int linearise(grille_t grille, coord_t p){
    return p.x + p.y*grille->largeur;
}

float minimum(float f1, float f2){
    if (f1 < f2) return f1;
    else return f2;
}

float dijkstra(
        grille_t grille, 
        coord_t source, coord_t destination, 
        float seuil,
        liste_noeud_t** chemin
    ) {
    liste_noeud_t* a_visiter = creer_liste();
    liste_noeud_t* precedants = creer_liste();
    bool * visite = malloc(sizeof(int) * grille->profondeur * grille->largeur);
    float* distances = malloc(sizeof(int) * grille->profondeur * grille->largeur);
    for (int k = 0; k < grille->profondeur * grille->largeur; k++){
        visite[k] = false;
        distances[k] = INFINITY;
    }

    distances[linearise(grille, source)] = 0.0;

    inserer_noeud_liste(a_visiter, source, source, 0);
    while (!est_vide_liste(a_visiter)){
        coord_t noeud_min = min_noeud_liste(a_visiter);
        supprimer_noeud_liste(a_visiter, noeud_min);
        visite[linearise(grille, noeud_min)] = true;
        coord_t* succ;

        int nb_succs = (int)get_voisins(grille, noeud_min, seuil, &succ);
        for (int i = 0; i<nb_succs; i++){
            coord_t noeud_suivant = succ[i];
            float prev_distance = distances[linearise(grille, noeud_suivant)];
            float nouv_distance = distances[linearise(grille, noeud_min)] + cout(grille, noeud_min, noeud_suivant);
            
            if (!visite[linearise(grille, noeud_suivant)] && prev_distance > nouv_distance){
                distances[linearise(grille, noeud_suivant)] = nouv_distance;
                inserer_noeud_liste(a_visiter, noeud_suivant, noeud_min, nouv_distance);
                inserer_noeud_liste(precedants, noeud_suivant, noeud_min, nouv_distance);
            }
        }
    }

    chemin = malloc(sizeof(liste_noeud_t*));
    construire_chemin_vers(chemin, precedants, source, destination);

    detruire_liste(a_visiter);
    detruire_liste(precedants);
    free(visite);
    free(distances);
}


