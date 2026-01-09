#ifndef LISTE_NOEUD.H
#define LISTE_NOEUD.H

#include <stdbool.h>
#include "coord.h"

struct liste_noeud_t;

typedef struct liste_noeud_t* liste_noeud_t;

/**
 * creer_liste : crée une liste de noeuds, initialement vide
 *
 * Post-conditions : `r = creer_liste()` => `r != NULL`, `est_vide_liste(r)`
 * @return liste nouvellement créée (de type liste_noeud_t)
 */
liste_noeud_t creer_liste(void);

/**
 * detruire_liste : détruit la liste passée en paramètre
 *
 * Pré-conditions : liste_ptr != NULL
 * Post-conditions : *liste_ptr == NULL
 *
 * @param liste_ptr pointeur sur la liste à détruire
 */
void detruire_liste(liste_noeud_t* liste_ptr);

/**
 * est_vide_liste : test si la liste passée en paramètre est vide
 *
 * Pré-conditions : liste != NULL
 *
 * @param liste [in] liste à tester
 * @return vrai ssi la liste ne contient aucun élément
 */
bool est_vide_liste(liste_noeud_t liste);

/**
 * contient_noeud_liste : test si le noeud donné appartient à la liste donnée.
 * 
 * Pré-conditions : liste != NULL
 *
 * @param liste [in] liste à parcourir
 * @param noeud noeud à rechercher
 * @return vrai ssi noeud est dans liste
 */
bool contient_noeud_liste(liste_noeud_t liste, coord_t noeud);

/**
 * contient_arrete_liste : test si l'arrête donnée appartient à la liste donnée.
 * L'arrête (source, destination) appartient à la liste ssi source et destination appartiennent 
 * à liste et si prec(destination) == source.
 *
 * Pré-conditions : liste != NULL
 *
 * @param liste [in] liste à parcourir
 * @param source noeud source de l'arrête
 * @param destination noeud destination de l'arrête
 * @return vrai ssi l'arrête (source,destination) est dans liste
 */
bool contient_arrete_liste(liste_noeud_t liste, coord_t source, coord_t destination);

/**
 * cout_noeud_liste : récupère le coût associé au noeud donné dans la liste donnée.
 * Si le noeud n'existe pas dans la liste, retourne `INFINITY`.
 *
 * Pré-conditions : liste != NULL
 * Post-conditions : `contient_noeud_liste(liste, noeud)` <=> `cout_noeud_liste(liste, noeud) != INFINITY`
 *
 * @param liste [in] liste à parcourir
 * @param noeud noeud dont on veut la distance
 * @return coût associé à noeud dans liste ou INFINITY si noeud n'est pas dans liste
 */
float cout_noeud_liste(liste_noeud_t liste, coord_t noeud);

/**
 * precedent_noeud_liste : récupère le noeud précédent associé au noeud donné dans la liste donnée.
 * Si le noeud n'existe pas, retourne des coordonnées négatives.
 * 
 * Pré-conditions : liste != NULL
 * Post-conditions : `contient_noeud_liste(liste, noeud)` 
 *                      => `get_x(precedent_noeud_liste(liste, noeud)) >= 0 &&`
 *                         `get_y(precedent_noeud_liste(liste, noeud)) >= 0`
 *
 * @param liste [in] liste à parcourir
 * @param noeud noeud dont on veut le précédent
 * @return précédent associé au noeud dans la liste (ou coordonnées négatives si le noeud n'existe pas)
 */
coord_t precedent_noeud_liste(liste_noeud_t liste, coord_t noeud);

/**
 * min_noeud_liste : trouve le (un) noeud de la liste dont le coût heuristique associé est le plus petit.
 *
 * Pré-conditions : liste non NULL, liste non vide
 * Post-conditions : `n = min_noeud_liste(liste)` =>
 *   pour tout `n', contient_noeud_liste(liste, n')`, `cout_heuristique_noeud_liste(liste, n) <= cout_heuristique_noeud_liste(liste, n')`
 *
 * @param liste [in] liste à parcourir
 * @return noeud qui minimise le coût heuristique
 */
coord_t min_noeud_liste(liste_noeud_t liste);

/**
 * inserer_noeud_liste : modifie les valeurs associées au noeud donné dans la liste donnée, et ajoute
 * le noeud s'il n'existe pas déjà.
 *
 * Pré-conditions : liste != NULL
 * Post-conditions :
 *   - `contient_noeud_liste(liste, noeud)`
 *   - `cout_noeud_liste(liste, noeud) == cout`
 *   - `precedent_noeud_liste(liste, noeud) == precedent`
 *
 * @param liste [in,out] liste à modifier
 * @param noeud noeud à modifier
 * @param precedent nouveau noeud précédent pour noeud
 * @param cout nouveau coût pour noeud
 */
void inserer_noeud_liste(liste_noeud_t liste, coord_t noeud, coord_t precedent, float cout);

/**
 * supprimer_noeud_liste : supprime le noeud donné de la liste. Si le noeud n'est pas dans la liste,
 * ne fait rien.
 *
 * Pré-conditions : liste != NULL
 * Post-conditions : après `supprimer_noeud_liste(liste, n)` : `!contient_noeud_liste(liste, n)`
 *
 * @param liste [in,out] liste à modifier
 * @param noeud noeud à supprimer de liste
 */
void supprimer_noeud_liste(liste_noeud_t liste, coord_t noeud);




#endif