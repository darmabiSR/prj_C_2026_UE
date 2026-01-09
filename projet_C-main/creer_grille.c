#define _GNU_SOURCE
#include "creer_grille.h"
#include "grille_file.h"
#include <math.h>

/**
 * Ajoute à la grille une surface gaussienne centrée en (x0,y0) de variance
 * horizontale varx et de variance verticale vary.
 *
 * Cette fonction permet concrèteement de faire des montagnes :D
 *
 * @param grille grille sur laquelle ajouter une gaussienne
 * @param x0 abscisse du sommet de la gaussienne
 * @param y0 ordonnée du sommet de la gaussienne
 * @param varx variance horizontale
 * @param vary variance verticale
 */
static void gauss(grille_t grille, float x0, float y0, float varx, float vary) {
    float dx, dy, h;
    for (int y = 0; y < get_profondeur(grille); y++) {
        for (int x = 0; x < get_largeur(grille); x++) {
            coord_t c = creer_coord(x, y);
            dx = ((float) x) - x0;
            dy = ((float) y) - y0;
            h = get_hauteur(grille, c) +
                expf(-( dx*dx/(2*varx*varx) + dy*dy/(2*vary*vary) ));
            set_hauteur(grille, c, h);
        }
    }
}

/**
 * remplir_grille : crée une grille sur laquelle exécuté l'algorithme de
 * Dijkstra. La grille a les mensurations que l'on veut, et contient des
 * nombres entre 0.0 et 1.0, ou potentiellement le nombre INFINITY, représenté
 * par un carré noir et correspondant à l'infini (un mur donc).
 *
 * La fonction prend en paramètre un chemin vers un fichier .grille, et c'est 
 * si ce chemin est NULL que la grille générée dans la fonction est utilisée.
 *
 * @param argv chemin du fichier grille à charger (potentiellement NULL)
 * @return grille chargée ou créée par la fonction
 */
grille_t remplir_grille(const char* argv) {
    grille_t grille;
    if (argv != NULL) {
        import_grille(&grille, argv);
    } else {
        /*
         | Mettez ici le code que vous voulez pour créer une grille customisée !
         | La grille doit contenir des nombres flottants de 0.0 à 1.0, ou le nombre
         | INFINITY qui représente un "mur".
         */
        // Exemple 1 : mur
        grille = creer_grille(7, 6);
        coord_t c;
        set_x(&c, 0); set_y(&c, 2); set_hauteur(grille, c, INFINITY);
        set_x(&c, 1); set_y(&c, 2); set_hauteur(grille, c, INFINITY);
        set_x(&c, 2); set_y(&c, 2); set_hauteur(grille, c, INFINITY);
        set_x(&c, 3); set_y(&c, 2); set_hauteur(grille, c, INFINITY);
        set_x(&c, 3); set_y(&c, 3); set_hauteur(grille, c, INFINITY);
        set_x(&c, 3); set_y(&c, 4); set_hauteur(grille, c, INFINITY);
        set_x(&c, 4); set_y(&c, 4); set_hauteur(grille, c, INFINITY);
        set_x(&c, 5); set_y(&c, 4); set_hauteur(grille, c, INFINITY);
        set_x(&c, 5); set_y(&c, 5); set_hauteur(grille, c, INFINITY);
        set_x(&c, 4); set_y(&c, 0); set_hauteur(grille, c, INFINITY);
        set_x(&c, 5); set_y(&c, 0); set_hauteur(grille, c, INFINITY);
        set_x(&c, 5); set_y(&c, 1); set_hauteur(grille, c, INFINITY);
        set_x(&c, 6); set_y(&c, 1); set_hauteur(grille, c, INFINITY);

        // Exemple 2 : des montagnes
        //grille = creer_grille(20, 20);
        //gauss(grille, 0.0, 5.0, 2.0, 2.0);
        //gauss(grille, 16.0, 5.5, 4.0, 3.5);
        //gauss(grille, 5.0, 14.0, 5.0, 3.0);
        
    }

    return grille;
}



