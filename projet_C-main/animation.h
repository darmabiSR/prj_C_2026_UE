#ifndef ANIMATION_H
#define ANIMATION_H

#include "app.h"
#include "grille.h"
#include "grille_view.h"
#include "liste_noeud.h"
#include <stdbool.h>

/**
 * Configure le module d'animation. Appelé une fois au début du programme.
 *
 * @param app application (nécessaire pour faire des actions graphiques)
 * @param view composant de visualisation de la grille
 */
void setup_animation(struct app_t* app, struct grille_view_t* view);

/**
 * Désactive les animations (les fonctions se contentent alors de se terminer
 * sans rien faire).
 *
 * Les animations sont désactivées par défaut.
 */
void disable_animation();

/**
 * Active les animations. Les appels de fonction entraînent un changement sur
 * la fenêtre.
 */
void enable_animation();

/**
 * Configure la durée de chaque animation. Un appel de fonction d'animation dure
 * environ cette durée.
 *
 * @param delay délai d'animation
 */
void set_animation_delay(unsigned int delay);

/**
 * Montre le noeud source sur le graphe (représenté par un petit point noir).
 * @param node noeud source à montrer
 */
void animate_source(coord_t node);

/**
 * Montre le noeud destination sur le graphe.
 * @param node noeud destination à montrer
 */
void animate_destination(coord_t node);

/**
 * Représente l'ensemble des noeuds visités (sous la forme de disques gris avec un trait vers
 * le noeud précédent de chaque noeud).
 * @param visited liste des noeuds visités
 */
void animate_visited(liste_noeud_t* visited);

/**
 * Représente le noeud actuellement en train d'être visité (sous la forme d'un disque vert).
 * @param node noeud en court de visite
 */
void animate_visiting(coord_t node);

/**
 * Représente le noeud voisin sous considération (sous la forme d'un disque rouge).
 * @param neighbor noeud voisin considéré
 */
void animate_neighbor(coord_t neighbor);

/**
 * Représente un (bout de) chemin (sous la forme de cercles reliés par des traits).
 * @param path chemin ou bout de chemin en cours de calcul
 */
void animate_path(liste_noeud_t* path);

/**
 * Retire le marquage du noeud source.
 */
void clear_source();

/**
 * Retire le marquage du noeud destination.
 */
void clear_destination();

/**
 * Retire le marquage des noeuds visités
 */
void clear_visited();

/**
 * Retire le marquage du noeud en cours de visite.
 */
void clear_visiting();

/**
 * Retire le marquage du noeud voisin en cours de considération.
 */
void clear_neighbor();

/**
 * Retire le marquage du chemin.
 */
void clear_path();

#endif



