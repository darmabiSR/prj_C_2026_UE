
#include "coord.h"
#include <math.h>

coord_t creer_coord(int x, int y){
    coord_t c;
    c.x = x;
    c.y = y;
    return c;
}

int get_x(coord_t coordonnee){
    return coordonnee.x;
}

int get_y(coord_t coordonnee){
    return coordonnee.y;
}

void set_x(coord_t* c, int x){
    (*c).x = x;
}

void set_y(coord_t* c, int y){
    (*c).y = y;
}

bool memes_coord(coord_t a, coord_t b){
    return (a.x==b.x) && (a.y==b.y);
}

coord_t translation(coord_t c, int dx, int dy){
    c.x += dx;
    c.y += dy;
    return c;

}

float distance_euclidienne(coord_t a, coord_t b){
    float dx = (float)(a.x - b.x);
    float dy = (float)(a.y - b.y);
    return sqrtf(dx*dx + dy*dy);
}