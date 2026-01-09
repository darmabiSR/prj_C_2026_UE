#ifndef GRILLE_VIEW_H
#define GRILLE_VIEW_H

#include <SDL2/SDL.h>
#include "grille.h"
#include "viewport.h"
#include "window.h"
#include "liste_noeud.h"

struct grille_view_t;

struct grille_view_t* create_grille_view(struct window_t* window, grille_t* grille, SDL_Rect area, viewport_t viewport);
struct grille_view_t* create_grille_view_vp(struct window_t* window, grille_t* grille, SDL_Rect area);

void dispose_grille_view(struct grille_view_t** view);
void get_viewport(const struct grille_view_t* view, viewport_t* result);

void move_mouse(struct grille_view_t* view, SDL_Point pos);

void highlight_path(struct grille_view_t* view, liste_noeud_t* path);
void highlight_start_node(struct grille_view_t* view, coord_t node);
void highlight_end_node(struct grille_view_t* view, coord_t node);
void highlight_current_node(struct grille_view_t* view, coord_t node);
void highlight_current_neighbor(struct grille_view_t* view, coord_t node);
void highlight_visited(struct grille_view_t* view, liste_noeud_t* visited);

void grille_view_repaint(struct window_t* window, void* userdata);

#endif 


