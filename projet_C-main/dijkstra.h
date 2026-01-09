// This file is part of dijkstra.
// 
// dijkstra is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// dijkstra is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR 
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with 
// dijkstra. If not, see <https://www.gnu.org/licenses/>.
//
// dijkstra - Copyright (c) 2024 Guillaume Dupont
// Contact: <guillaume.dupont@toulouse-inp.fr>
/**
 * @file dijkstra.h
 * @brief Module dijkstra qui contient la fonction pour réaliser l'algorithme de dijkstra
 *
 * @author G. Dupont
 * @version 1.0
 */
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "grille.h"
#include "liste_noeud.h"

/**
 * dijkstra - Calcul le plus court (= de moindre coût) chemin dans un graphe représenté par une grille, 
 * entre les noeuds donnés.
 * La fonction retourne le coût calculé, et peuple le chemin passé en paramètre
 * (si non NULL) avec le chemin correspondant.
 *
 * Pré-conditions : 
 *   - source référence un noeud valide de la grille
 *   - destination référence un noeud valide de la grille
 *   - source et destination sont connectés (il existe une succession de voisins de source à destination)
 * Post-conditions :
 *   - retour >= 0.0
 *   - si chemin != NULL, *chemin contient un chemin connexe de source à destination
 *     La fonction se charge d'allouer le chemin sur le tas (et l'utilisateur se charge de le
 *     détruire).
 *
 * @param grille graphe dans lequel calculer le chemin
 * @param source noeud de départ (coordonnées dans la grille)
 * @param destination noeud de destination (coordonnées dans la grille)
 * @param chemin [out] pointeur sur une variable de type liste_noeud_t* pour recevoir
 *   le chemin calculé
 * @return coût minimal entre le noeud source et destination, en suivant la grille
 */
float dijkstra(
        grille_t grille, 
        coord_t source, coord_t destination, 
        float seuil,
        liste_noeud_t** chemin
    );



#endif // DIJKSTRA_H


