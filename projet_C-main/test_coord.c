#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "coord.h"
#include "tests.h"

#define PRECISION 1e-4

int main() {
    INITIALIZE_TESTS();
    SET_ANALYSIS("test_coord.csv");

    BEGIN_SECTION("coord/base")
        BEGIN_TESTI("cretation-access")
            coord_t c = creer_coord(11, -5);
            ASSERT_EQ(get_x(c), 11);
            ASSERT_EQ(get_y(c), -5);
        END_TEST

        BEGIN_TESTI("set_x")
            coord_t c = creer_coord(22, 31);
            set_x(&c, -18);
            ASSERT_EQ(get_x(c), -18);
            ASSERT_EQ(get_y(c),  31);
        END_TEST

        BEGIN_TESTI("set_y")
            coord_t c = creer_coord(61, 87);
            set_y(&c, 91);
            ASSERT_EQ(get_x(c), 61);
            ASSERT_EQ(get_y(c), 91);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("coord/comparaison")
        BEGIN_TESTI("memes_coord-true")
            coord_t c1 = creer_coord(21, 11);
            coord_t c2 = creer_coord(21, 11);
            ASSERT(memes_coord(c1, c2));
        END_TEST

        BEGIN_TESTI("memes_coord-false")
            coord_t c1 = creer_coord(21, 11);
            coord_t c2 = creer_coord(11, 11);
            ASSERT(!memes_coord(c1, c2));
        END_TEST

        BEGIN_TESTI("memes_coord-false-2")
            coord_t c1 = creer_coord(11, 11);
            coord_t c2 = creer_coord(11, 21);
            ASSERT(!memes_coord(c1, c2));
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("coord/translation")
        BEGIN_TESTI("translation-0")
            coord_t c = creer_coord(10, 15);
            coord_t ct = translation(c, 0, 0);
            ASSERT(memes_coord(c, ct));
        END_TEST

        BEGIN_TESTI("translation-x")
            coord_t c = creer_coord(11, -16);
            coord_t ct = translation(c, -5, 0);
            ASSERT_EQ(get_x(ct), 6);
            ASSERT_EQ(get_y(ct), -16);
        END_TEST

        BEGIN_TESTI("translation-y")
            coord_t c = creer_coord(-7, 21);
            coord_t ct = translation(c, 0, 5);
            ASSERT_EQ(get_x(ct), -7);
            ASSERT_EQ(get_y(ct), 26);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    BEGIN_SECTION("coord/distance")
        BEGIN_TESTI("distance-0")
            coord_t c1 = creer_coord(2, 3);
            ASSERT_EQ_F(distance_euclidienne(c1, c1), 0.0, PRECISION);
        END_TEST

        BEGIN_TESTI("distance-1")
            coord_t c1 = creer_coord(5, 4);
            coord_t c2 = creer_coord(5, 2);
            ASSERT_EQ_F(distance_euclidienne(c1, c2), 2.0, PRECISION);
        END_TEST

        BEGIN_TESTI("distance-2")
            coord_t c1 = creer_coord(2, -5);
            coord_t c2 = creer_coord(2, 2);
            ASSERT_EQ_F(distance_euclidienne(c1, c2), 7.0, PRECISION);
        END_TEST

        REPORT_TO_STDOUT;
        REPORT_ANALYSIS;
    END_SECTION()

    END_ANALYSIS;

    return 0;
}



