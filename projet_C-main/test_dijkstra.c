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
 * @file test_dijkstra.c
 * @brief Tests unitaires pour l'algorithme de dijkstra
 *
 * @author G. Dupont
 * @version 1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include "tests.h"
#include "coord.h"
#include "grille.h"
#include "liste_noeud.h"
#include "dijkstra.h"
#include "animation.h"

#define PRECISION   1e-4
#define TIMEOUT     50 _Millisec

/**
 * check_chemin - Fonction type ASSERT custom pour vérifier qu'un chemin se compose bien des 
 * noeuds donnés.
 *
 * Les noeuds donnés sont sous la forme d'une liste d'arguments (variadique) où chaque argument
 * est un identifiant de noeud, et terminant par NO_ID.
 *
 * La fonction vérifie que chaque noeud est bien dans la liste, et que les successions de noeuds
 * correspondent bien à la configuration (précédent(x), x) dans la liste. Pour le premier noeud,
 * on vérifie que le précédent est NO_ID.
 *
 * Si le chemin ne correspond pas, échoue le test actuel.
 *
 * @param graphe graphe dans lequel le chemin doit évoluer
 * @param chemin chemin à tester
 * @params liste des noeuds se terminant par NO_ID
 *
 */
static void ASSERT_FUN(check_chemin, const liste_noeud_t* chemin, ...) {
    va_list attendus;
    va_start(attendus, chemin);
    coord_t attendu, precedent;
    bool premiertour = true;
    while ((attendu.x = va_arg(attendus, int)) > 0 && (attendu.y = va_arg(attendus, int)) > 0) {
        if (!contient_noeud_liste(chemin, attendu)) {
            FAILF("Le chemin ne contient pas le noeud <<(%d, %d)>>\n", 
                    get_x(attendu), get_y(attendu));
        }
        if (premiertour) {
        } else {
            coord_t prec = precedent_noeud_liste(chemin, attendu);
            if (!memes_coord(prec, precedent)) {
                FAILF("Le précédent du noeud <<(%d,%d)>> est <<(%d,%d)>>, il devrait être le noeud <<(%d,%d)>>\n",
                        get_x(attendu), get_y(attendu),
                        get_x(prec), get_y(prec),
                        get_x(precedent), get_y(precedent));
            }
        }
        precedent = attendu;
    }
}

static grille_t make_grille(int largeur, int profondeur, float valeurs[]) {
    grille_t grille = creer_grille(largeur, profondeur);
    for (int x = 0; x < largeur; x++) {
        for (int y = 0; y < profondeur; y++) {
            coord_t c = creer_coord(x, y);
            set_hauteur(grille, c, valeurs[x * largeur + y]);
        }
    }
    return grille;
}

static grille_t make_grille2(int largeur, int profondeur, ...) {
    va_list valeurs;
    va_start(valeurs, profondeur);
    float valeur;
    grille_t grille = creer_grille(largeur, profondeur);

    for (int x = 0; x < largeur; x++) {
        for (int y = 0; y < profondeur; y++) {
            valeur = va_arg(valeurs, double);
            coord_t c = creer_coord(x, y);
            set_hauteur(grille, c, valeur);
        }
    }
    return grille;
}

int main() {
    disable_animation(); // Au cas où

    INITIALIZE_TESTS();
    SET_ANALYSIS("test_dijkstra.csv");

    BEGIN_SECTION("dijkstra")
        BEGIN_TESTI("dijkstra-1x1")
            SET_TIMEOUT(TIMEOUT);
            grille_t grille = make_grille2(1, 1, 0.0);
            coord_t c = creer_coord(0, 0);
            liste_noeud_t* chemin;

            float dist = dijkstra(grille, c, c, 1.0, &chemin);

            ASSERT_EQ_F(dist, 0.f, PRECISION);
            CALL_ASSERT_FUN(check_chemin, chemin, 0, 0, -1);

            detruire_liste(&chemin);
            detruire_grille(grille);
        END_TEST

        BEGIN_TESTI("dijkstra-1x1-no-path")
            SET_TIMEOUT(TIMEOUT);
            grille_t grille = make_grille2(1, 1, 0.0);
            coord_t c = creer_coord(0, 0);

            float dist = dijkstra(grille, c, c, 1.0, NULL);

            ASSERT_EQ_F(dist, 0.f, PRECISION);

            detruire_grille(grille);
        END_TEST

        BEGIN_TESTI("dijkstra-2x2")
            SET_TIMEOUT(TIMEOUT);
            /*
              #.
              ..
             */
            grille_t grille = make_grille2(2, 2, 
                    0.0, 0.0, 
                    100.0, 0.0);
            coord_t start = creer_coord(0, 0);
            coord_t dest = creer_coord(1, 1);
            liste_noeud_t* chemin;

            float dist = dijkstra(grille, start, dest, 1.0, &chemin);
            ASSERT_EQ_F(dist, 2.f, PRECISION);
            CALL_ASSERT_FUN(check_chemin, chemin,
                    0, 0,
                    1, 0,
                    1, 1,
                    -1);
            detruire_liste(&chemin);
            detruire_grille(grille);
        END_TEST

        BEGIN_TESTI("dijkstra-2x2-peak")
            SET_TIMEOUT(TIMEOUT);
            /*
              #.
              .O
             */
            grille_t grille = make_grille2(2, 2, 
                    0.0, 0.5, 
                    100.0, 0.0);
            coord_t start = creer_coord(0, 0);
            coord_t dest = creer_coord(1, 1);
            liste_noeud_t* chemin;

            float dist = dijkstra(grille, start, dest, 1.0, &chemin);
            ASSERT_EQ_F(dist, 2.5f, PRECISION);
            CALL_ASSERT_FUN(check_chemin, chemin,
                    0, 0,
                    1, 0,
                    1, 1,
                    -1);
            detruire_liste(&chemin);
            detruire_grille(grille);
        END_TEST

        BEGIN_TESTI("dijkstra-2x2-peak-start")
            SET_TIMEOUT(TIMEOUT);
            /*
              #.
              O.
             */
            grille_t grille = make_grille2(2, 2, 
                    0.5, 0.0, 
                    100.0, 0.0);
            coord_t start = creer_coord(0, 0);
            coord_t dest = creer_coord(1, 1);
            liste_noeud_t* chemin;

            float dist = dijkstra(grille, start, dest, 1.0, &chemin);
            ASSERT_EQ_F(dist, 2.f, PRECISION);
            CALL_ASSERT_FUN(check_chemin, chemin,
                    0, 0,
                    1, 0,
                    1, 1,
                    -1);
            detruire_liste(&chemin);
            detruire_grille(grille);
        END_TEST

        BEGIN_TESTI("dijkstra-3x3")
            SET_TIMEOUT(TIMEOUT)
            /*
               O..
               OoO
               ..O
             */
            grille_t grille = make_grille2(3, 3,
                    0.0, 0.0, 0.7, 
                    0.7, 0.3, 0.7, 
                    0.7, 0.0, 0.0);
            coord_t start = creer_coord(0, 0);
            coord_t dest = creer_coord(2, 2);
            liste_noeud_t* chemin;

            float dist = dijkstra(grille, start, dest, 1.0, &chemin);
            ASSERT_EQ_F(dist, 4.3f, PRECISION);
            CALL_ASSERT_FUN(check_chemin, chemin,
                    0, 0,
                    1, 0,
                    1, 1, 
                    1, 2,
                    2, 2,
                    -1);
            detruire_liste(&chemin);
            detruire_grille(grille);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    END_ANALYSIS;

    return 0;
}

