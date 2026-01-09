
#ifndef COORD_H
#define COORD_H
#include <stdbool.h>

struct coord_t {
    int x;
    int y;
};
typedef struct coord_t coord_t;

/* fonction qui crée une coordonnée*/
coord_t creer_coord(int x, int y);

/* fonction qui retourne l'abscisse d'une coordonnée*/
int get_x(coord_t coordonnee);

/* fonction qui retourne l'ordonnée d'une coordonnée*/
int get_y(coord_t coordonnee);

/* procédure qui change l'abscisse d'une coordonnée*/
void set_x(coord_t* coordonnee, int x);

/* procédure qui change l'ordonnée d'une coordonnée*/
void set_y(coord_t* coordonee, int y);

/* fonction qui vérifie si deux coordonnées sont les mêmes*/
bool memes_coord(coord_t a, coord_t b);

/* fonction qui translate une coordonnée*/
coord_t translation(coord_t coordonnee, int dx, int dy);

/* fonction qui calcule la distance euclidienne de deux coordonnées*/
float distance_euclidienne(coord_t a, coord_t b);

#endif

