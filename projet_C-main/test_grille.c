#include <stdio.h>
#include <stdlib.h>
#include "coord.h"
#include "grille.h"
#include "tests.h"

#define PRECISION   1e-4

static void ASSERT_FUN(parmi, coord_t c, coord_t* tab, size_t taille) {
    bool trouve = false;
    for (int i = 0; i < taille && !trouve; i++) {
        if (memes_coord(c, tab[i])) {
            trouve = true;
        }
    }

    if (!trouve) {
        FAILF("Le tableau ne contient pas les coordonnées <<(%d,%d)>>\n",
                get_x(c), get_y(c));
    }
}

int main() {
    INITIALIZE_TESTS();
    SET_ANALYSIS("test_grille.csv");

    BEGIN_SECTION("grille/base")
        BEGIN_TESTI("grille-creation")
            grille_t g = creer_grille(100, 200);
            ASSERT_EQ(get_largeur(g), 100);
            ASSERT_EQ(get_profondeur(g), 200);
            for (int i = 0; i < 100; i++) {
                for (int j = 0; j < 100; j++) {
                    coord_t c = creer_coord(i, j);
                    ASSERT_EQ_F(get_hauteur(g, c), 0.f, PRECISION);
                }
            }
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("grille-access")
            grille_t g = creer_grille(50, 70);
            set_hauteur(g, creer_coord(0, 10), 27.f);
            ASSERT_EQ_F(get_hauteur(g, creer_coord(0, 10)), 27.f, PRECISION);
            for (int i = 0; i < 50; i++) {
                for (int j = 0; j < 70; j++) {
                    if (!(i == 0 && j == 10)) {
                        coord_t c = creer_coord(i, j);
                        ASSERT_EQ_F(get_hauteur(g, c), 0.f, PRECISION);
                    }
                }
            }
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("grille-acces2")
            grille_t g = creer_grille(250, 110);
            set_hauteur(g, creer_coord(249, 109), 125.f);
            ASSERT_EQ_F(get_hauteur(g, creer_coord(249, 109)), 125.f, PRECISION);
            detruire_grille(g);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("grille/dans_les_bornes")
        BEGIN_TESTI("dans_les_bornes-oui-1")
            grille_t g = creer_grille(4, 3);
            ASSERT(dans_les_bornes(g, creer_coord(2, 1)));
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("dans_les_bornes-oui-2")
            grille_t g = creer_grille(4, 3);
            ASSERT(dans_les_bornes(g, creer_coord(0, 0)));
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("dans_les_bornes-non-1")
            grille_t g = creer_grille(4, 3);
            ASSERT(!dans_les_bornes(g, creer_coord(12, 51)));
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("dans_les_bornes-non-2")
            grille_t g = creer_grille(4, 3);
            ASSERT(!dans_les_bornes(g, creer_coord(4, 2)));
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("dans_les_bornes-non-3")
            grille_t g = creer_grille(4, 3);
            ASSERT(!dans_les_bornes(g, creer_coord(0, 3)));
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("dans_les_bornes-non-4")
            grille_t g = creer_grille(4, 3);
            ASSERT(!dans_les_bornes(g, creer_coord(-2, -3)));
            detruire_grille(g);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("grille/points")
        BEGIN_TESTI("inferieur_gauche")
            grille_t g = creer_grille(5, 6);
            coord_t c = inferieur_gauche(g);
            ASSERT_EQ(get_x(c), 0);
            ASSERT_EQ(get_y(c), 0);
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("superieur_droit-1")
            grille_t g = creer_grille(5, 6);
            coord_t c = superieur_droit(g);
            ASSERT_EQ(get_x(c), 4);
            ASSERT_EQ(get_y(c), 5);
            detruire_grille(g);
        END_TEST

        BEGIN_TESTI("superieur_droit-2")
            grille_t g = creer_grille(11, 20);
            coord_t c = superieur_droit(g);
            ASSERT_EQ(get_x(c), 10);
            ASSERT_EQ(get_y(c), 19);
            detruire_grille(g);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("grille/voisins")
        BEGIN_TESTI("tous_voisins")
            grille_t g = creer_grille(4, 4);
            coord_t* voisins;
            size_t n = get_voisins(g, creer_coord(2, 2), 1.0, &voisins);
            CALL_ASSERT_FUN(parmi, creer_coord(1, 2), voisins, n);
            CALL_ASSERT_FUN(parmi, creer_coord(2, 1), voisins, n);
            CALL_ASSERT_FUN(parmi, creer_coord(3, 2), voisins, n);
            CALL_ASSERT_FUN(parmi, creer_coord(2, 3), voisins, n);
            ASSERT_EQ(n, 4);
            detruire_grille(g);
            free(voisins);
        END_TEST

        BEGIN_TESTI("inferieur_gauche")
            grille_t g = creer_grille(4, 4);
            coord_t* voisins;
            size_t n = get_voisins(g, creer_coord(0, 0), 1.0, &voisins);
            CALL_ASSERT_FUN(parmi, creer_coord(0, 1), voisins, n);
            CALL_ASSERT_FUN(parmi, creer_coord(1, 0), voisins, n);
            ASSERT_EQ(n, 2);
            detruire_grille(g);
            free(voisins);
        END_TEST

        BEGIN_TESTI("superieur_gauche")
            grille_t g = creer_grille(4, 4);
            coord_t* voisins;
            size_t n = get_voisins(g, creer_coord(0, 3), 1.0, &voisins);
            CALL_ASSERT_FUN(parmi, creer_coord(1, 3), voisins, n);
            CALL_ASSERT_FUN(parmi, creer_coord(0, 2), voisins, n);
            ASSERT_EQ(n, 2);
            detruire_grille(g);
            free(voisins);
        END_TEST

        BEGIN_TESTI("tous_voisins-seuil")
            grille_t g = creer_grille(4, 4);
            set_hauteur(g, creer_coord(1, 2), 100.0);
            set_hauteur(g, creer_coord(2, 1), 0.5);
            set_hauteur(g, creer_coord(3, 2), 100.0);
            coord_t* voisins;
            size_t n = get_voisins(g, creer_coord(2, 2), 1.0, &voisins);
            CALL_ASSERT_FUN(parmi, creer_coord(2, 1), voisins, n);
            CALL_ASSERT_FUN(parmi, creer_coord(2, 3), voisins, n);
            ASSERT_EQ(n, 2);
            detruire_grille(g);
            free(voisins);
        END_TEST


        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    END_ANALYSIS;

    return 0;
}



