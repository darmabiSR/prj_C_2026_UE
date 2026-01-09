#include "animation.h"


void setup_animation(struct app_t* app, struct grille_view_t* view) { (void) app; (void) view; }

void disable_animation() {}
void enable_animation() {}

void set_animation_delay(unsigned int delay) { (void) delay; }

void animate_source(coord_t node) { (void) node; }
void animate_destination(coord_t node) { (void) node; }
void animate_visited(liste_noeud_t* visited) { (void) visited; }
void animate_visiting(coord_t node) { (void) node; }
void animate_neighbor(coord_t neighbor) { (void) neighbor; }
void animate_path(liste_noeud_t* path) { (void) path; }

void clear_source() {}
void clear_destination() {}
void clear_visited() {}
void clear_visiting() {}
void clear_neighbor() {}
void clear_path() {}


