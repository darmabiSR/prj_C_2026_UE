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
 * @file test_liste_noeud.c
 * @brief Fichier de tests unitaires pour le module `liste_noeud`
 *
 * @author G. Dupont
 * @version 1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "tests.h"
#include "grille.h"
#include "coord.h"
#include "liste_noeud.h"

#define PRECISION   1e-4

typedef int noeud_id_t;

static bool contient_noeud_liste_wrap(const liste_noeud_t* liste, int noeud) {
    return contient_noeud_liste(liste, creer_coord(noeud, noeud));
}

static bool contient_arrete_liste_wrap(const liste_noeud_t* liste, int src, int dest) {
    return contient_arrete_liste(liste, creer_coord(src, src), creer_coord(dest, dest));
}

static float cout_noeud_liste_wrap(const liste_noeud_t* liste, int noeud) {
    return cout_noeud_liste(liste, creer_coord(noeud, noeud));
}

static int precedent_noeud_liste_wrap(const liste_noeud_t* liste, int noeud) {
    return get_x(precedent_noeud_liste(liste, creer_coord(noeud, noeud)));
}

static int min_noeud_liste_wrap(const liste_noeud_t* liste) {
    return get_x(min_noeud_liste(liste));
}

static void inserer_noeud_liste_wrap(liste_noeud_t* liste, int noeud, int prec, float cout) {
    inserer_noeud_liste(liste, creer_coord(noeud, noeud), creer_coord(prec, prec), cout);
}

static void supprimer_noeud_liste_wrap(liste_noeud_t* liste, int noeud) {
    supprimer_noeud_liste(liste, creer_coord(noeud, noeud));
}

int main() {
    INITIALIZE_TESTS();
    SET_ANALYSIS("test_liste_noeud.csv");

    BEGIN_SECTION("liste_noeud/creation-destruction")
        BEGIN_TESTI("creation/destruction")
            liste_noeud_t* t = creer_liste();
            detruire_liste(&t);
            ASSERT(t == NULL);
        END_TEST

        BEGIN_TESTI("creation/insertion/destruction")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 1, 1.0);
            inserer_noeud_liste_wrap(t, 2, 1, 1.0);
            detruire_liste(&t);
            ASSERT(t == NULL);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/inserer")
        BEGIN_TESTI("inserer-1")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 5, 1.0);
            ASSERT(contient_noeud_liste_wrap(t, 1));
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 1), 1.0, PRECISION);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 1), 5);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("inserer-2")
            liste_noeud_t* t = creer_liste();

            inserer_noeud_liste_wrap(t, 1, 5, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, -2.0);
            inserer_noeud_liste_wrap(t, 3, 2, -5.0);
            inserer_noeud_liste_wrap(t, 4, 1, 7.0);
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 2), -2.0, PRECISION);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 2), 7);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("changer_noeud-debut")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, 1.5);
            inserer_noeud_liste_wrap(t, 3, 8, 1.6);

            inserer_noeud_liste_wrap(t, 1, 10, 10.0);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 1), 10.0, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 2), 1.5, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 3), 1.6, PRECISION);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 1), 10);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 2), 7);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 3), 8);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("changer_noeud-milieu")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, 1.5);
            inserer_noeud_liste_wrap(t, 3, 8, 1.6);

            inserer_noeud_liste_wrap(t, 2, 10, 10.0);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 1), 1.0, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 2), 10.0, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 3), 1.6, PRECISION);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 1), 6);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 2), 10);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 3), 8);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("changer_noeud-fin")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, 1.5);
            inserer_noeud_liste_wrap(t, 3, 8, 1.6);

            inserer_noeud_liste_wrap(t, 3, 10, 10.0);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 1), 1.0, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 2), 1.5, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 3), 10.0, PRECISION);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 1), 6);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 2), 7);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 3), 10);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("changer_noeud-nouveau")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, 1.5);
            inserer_noeud_liste_wrap(t, 3, 8, 1.6);
            ASSERT(!contient_noeud_liste_wrap(t, 4));

            inserer_noeud_liste_wrap(t, 4, 10, 10.0);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 1), 1.0, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 2), 1.5, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 3), 1.6, PRECISION);
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 4), 10.0, PRECISION);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 1), 6);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 2), 7);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 3), 8);
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 4), 10);
            detruire_liste(&t);
        END_TEST


        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/contient")
        liste_noeud_t* t = creer_liste();
        inserer_noeud_liste_wrap(t, 1, 10, 1.0);
        inserer_noeud_liste_wrap(t, 2, 11, 2.0);
        inserer_noeud_liste_wrap(t, 3, 12, 3.0);
        inserer_noeud_liste_wrap(t, 4, 13, 4.0);

        BEGIN_TESTI("contient")
            ASSERT(contient_noeud_liste_wrap(t, 1));
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            ASSERT(contient_noeud_liste_wrap(t, 4));
            ASSERT(!contient_noeud_liste_wrap(t, 10));
        END_TEST

        detruire_liste(&t);

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/cout")
        liste_noeud_t* t = creer_liste();
        inserer_noeud_liste_wrap(t, 1, 10, 1.0);
        inserer_noeud_liste_wrap(t, 2, 11, 2.0);
        inserer_noeud_liste_wrap(t, 3, 12, 3.0);
        inserer_noeud_liste_wrap(t, 4, 13, 4.0);

        BEGIN_TESTI("cout-debut")
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 1), 1.0, PRECISION);
        END_TEST

        BEGIN_TESTI("cout-milieu")
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 3), 3.0, PRECISION);
        END_TEST

        BEGIN_TESTI("cout-fin")
            ASSERT_EQ_F(cout_noeud_liste_wrap(t, 4), 4.0, PRECISION);
        END_TEST

        BEGIN_TESTI("cout-non-contient")
            ASSERT_EQ(cout_noeud_liste_wrap(t, 10), INFINITY);
        END_TEST

        detruire_liste(&t);

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/precedent")
        liste_noeud_t* t = creer_liste();
        inserer_noeud_liste_wrap(t, 1, 10, 1.0);
        inserer_noeud_liste_wrap(t, 2, 11, 2.0);
        inserer_noeud_liste_wrap(t, 3, 12, 3.0);
        inserer_noeud_liste_wrap(t, 4, 13, 4.0);

        BEGIN_TESTI("precedent-debut")
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 1), 10);
        END_TEST

        BEGIN_TESTI("precedent-milieu")
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 2), 11);
        END_TEST

        BEGIN_TESTI("precedent-fin")
            ASSERT_EQ(precedent_noeud_liste_wrap(t, 4), 13);
        END_TEST

        detruire_liste(&t);

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/min")
        BEGIN_TESTI("min-debut")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, 1.5);
            inserer_noeud_liste_wrap(t, 3, 8, 1.7);
            
            noeud_id_t ret = min_noeud_liste_wrap(t);
            ASSERT_EQ(ret, 1);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("min-milieu")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, 1.5);
            inserer_noeud_liste_wrap(t, 3, 8, -0.1);
            inserer_noeud_liste_wrap(t, 4, 9, 1.7);
            
            noeud_id_t ret = min_noeud_liste_wrap(t);
            ASSERT_EQ(ret, 3);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("min-fin")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 7, 1.5);
            inserer_noeud_liste_wrap(t, 3, 8, -0.1);
            
            noeud_id_t ret = min_noeud_liste_wrap(t);
            ASSERT_EQ(ret, 3);
            detruire_liste(&t);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/contient_arrete")
        liste_noeud_t* t = creer_liste();
        inserer_noeud_liste_wrap(t, 1, 2, 1.0);
        inserer_noeud_liste_wrap(t, 2, 3, 1.0);
        inserer_noeud_liste_wrap(t, 4, 1, 1.0);

        BEGIN_TESTI("contient_arrete-oui-1")
            ASSERT(contient_arrete_liste_wrap(t, 2, 1)); // pred(1) == 2
        END_TEST

        BEGIN_TESTI("contient_arrete-oui-2")
            ASSERT(contient_arrete_liste_wrap(t, 1, 4)); // pred(4) == 1
        END_TEST

        BEGIN_TESTI("contient_arrete-non-1")
            ASSERT(!contient_arrete_liste_wrap(t, 5, 6));
        END_TEST

        BEGIN_TESTI("contient_arrete-non-2")
            ASSERT(!contient_arrete_liste_wrap(t, 3, 1));
        END_TEST

        BEGIN_TESTI("contient_arrete-non-3")
            ASSERT(!contient_arrete_liste_wrap(t, 2, 3));
        END_TEST

        detruire_liste(&t);
        
        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/supprimer")
        BEGIN_TESTI("supprimer-debut")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 6, 1.0);
            inserer_noeud_liste_wrap(t, 3, 6, 1.0);
            supprimer_noeud_liste_wrap(t, 1);
            ASSERT(!contient_noeud_liste_wrap(t, 1));
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("supprimer-milieu")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 6, 1.0);
            inserer_noeud_liste_wrap(t, 3, 6, 1.0);
            inserer_noeud_liste_wrap(t, 4, 6, 1.0);
            supprimer_noeud_liste_wrap(t, 3);
            ASSERT(contient_noeud_liste_wrap(t, 1));
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(!contient_noeud_liste_wrap(t, 3));
            ASSERT(contient_noeud_liste_wrap(t, 4));
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("supprimer-fin")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 6, 1.0);
            inserer_noeud_liste_wrap(t, 3, 6, 1.0);
            supprimer_noeud_liste_wrap(t, 3);
            ASSERT(contient_noeud_liste_wrap(t, 1));
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(!contient_noeud_liste_wrap(t, 3));
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("supprimer-zero")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 6, 1.0);
            inserer_noeud_liste_wrap(t, 2, 6, 1.0);
            inserer_noeud_liste_wrap(t, 3, 6, 1.0);
            supprimer_noeud_liste_wrap(t, 4);
            ASSERT(contient_noeud_liste_wrap(t, 1));
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("supprimer-vide")
            liste_noeud_t* t = creer_liste();
            supprimer_noeud_liste_wrap(t, 5);
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("insertion-suppression")
            liste_noeud_t* t = creer_liste();

            inserer_noeud_liste_wrap(t, 1, 0, 0.0);
            inserer_noeud_liste_wrap(t, 2, 0, 1.0);
            inserer_noeud_liste_wrap(t, 3, 0, 1.2);
            inserer_noeud_liste_wrap(t, 4, 0, 0.7);

            ASSERT(contient_noeud_liste_wrap(t, 1));
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            ASSERT(contient_noeud_liste_wrap(t, 4));

            supprimer_noeud_liste_wrap(t, 1);

            ASSERT(!contient_noeud_liste_wrap(t, 1));
            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            ASSERT(contient_noeud_liste_wrap(t, 4));

            supprimer_noeud_liste_wrap(t, 4);

            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            ASSERT(!contient_noeud_liste_wrap(t, 4));

            inserer_noeud_liste_wrap(t, 5, 0, 1.1);

            ASSERT(contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            ASSERT(contient_noeud_liste_wrap(t, 5));

            supprimer_noeud_liste_wrap(t, 2);

            ASSERT(!contient_noeud_liste_wrap(t, 2));
            ASSERT(contient_noeud_liste_wrap(t, 3));
            ASSERT(contient_noeud_liste_wrap(t, 5));

            supprimer_noeud_liste_wrap(t, 5);

            ASSERT(contient_noeud_liste_wrap(t, 3));
            ASSERT(!contient_noeud_liste_wrap(t, 5));

            supprimer_noeud_liste_wrap(t, 3);

            ASSERT(est_vide_liste(t));

            detruire_liste(&t);
        END_TEST


        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("liste_noeud/est_vide")
        BEGIN_TESTI("est_vide-0")
            liste_noeud_t* t = creer_liste();
            ASSERT(est_vide_liste(t));
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("est_vide-1")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 1, 1.0);
            ASSERT(!est_vide_liste(t));
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("est_vide-2")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 1, 1.0);
            supprimer_noeud_liste_wrap(t, 1);
            ASSERT(est_vide_liste(t));
            detruire_liste(&t);
        END_TEST

        BEGIN_TESTI("est_vide-3")
            liste_noeud_t* t = creer_liste();
            inserer_noeud_liste_wrap(t, 1, 1, 1.0);
            supprimer_noeud_liste_wrap(t, 1);
            inserer_noeud_liste_wrap(t, 1, 1, 1.0);
            supprimer_noeud_liste_wrap(t, 1);
            ASSERT(est_vide_liste(t));
            detruire_liste(&t);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    END_ANALYSIS;

    return 0;
}


