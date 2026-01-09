#define _GNU_SOURCE
#include "grille_view.h"
#include <math.h>
#include "viewport.h"
#include "window.h"
#include "util.h"
#include "liste_noeud.h"

#define NODE_RADIUS     0.4
#define CLAMP_UINT8(x)    ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))

static point_t coord_to_point(coord_t c) {
    point_t p = { .x = (double) get_x(c), .y = (double) get_y(c) };
    return p;
}

struct grille_view_t {
    viewport_t viewport;
    SDL_Rect area;

    grille_t* grille;

    liste_noeud_t* path;
    coord_t startNode;
    coord_t endNode;
    coord_t currentNode;
    coord_t neighboringNode;
    liste_noeud_t* visited;

    struct window_t* window;
    SDL_mutex* mutex;
};

struct grille_view_t* create_grille_view(struct window_t* window, grille_t* grille, SDL_Rect area, viewport_t viewport) {
    struct grille_view_t* result = (struct grille_view_t*) malloc(sizeof(struct grille_view_t));
    result->grille = grille;
    copy_viewport(&(result->viewport), viewport);
    result->area.x = area.x;
    result->area.y = area.y;
    result->area.w = area.w;
    result->area.h = area.h;
    result->path = NULL;
    result->startNode = creer_coord(-1, -1);
    result->endNode = creer_coord(-1, -1);
    result->currentNode = creer_coord(-1, -1);
    result->neighboringNode = creer_coord(-1, -1);
    result->visited = NULL;
    result->window = window;
    result->mutex = SDL_CreateMutex();
    return result;
}

struct grille_view_t* create_grille_view_vp(struct window_t* window, grille_t* grille, SDL_Rect area) {
    rect_t vparea;
    coord_t cbl = inferieur_gauche(*grille);
    coord_t ctr = superieur_droit(*grille);
    point_t pbl = { .x = (double) get_x(cbl) - 0.5, .y = (double) get_y(cbl) - 0.5 };
    point_t ptr = { .x = (double) get_x(ctr) + 0.5, .y = (double) get_y(ctr) + 0.5 };
    mkrect_corners(&vparea, pbl, ptr);
    
    viewport_t viewport;
    viewport_unit_fit(&viewport, area, vparea);

    return create_grille_view(window, grille, area, viewport);
}

void dispose_grille_view(struct grille_view_t** view) {
    SDL_DestroyMutex((*view)->mutex);
    free(*view);
    *view = NULL;
}

void get_viewport(const struct grille_view_t* view, viewport_t* result) {
    copy_viewport(result, view->viewport);
}

void move_mouse(struct grille_view_t* view, SDL_Point pos) {
    (void)view;
    (void)pos;
}

void highlight_path(struct grille_view_t* view, liste_noeud_t* path) {
    SDL_LockMutex(view->mutex);
    view->path = path;
    SDL_UnlockMutex(view->mutex);
}

void highlight_start_node(struct grille_view_t* view, coord_t node) {
    SDL_LockMutex(view->mutex);
    view->startNode = node;
    SDL_UnlockMutex(view->mutex);
}

void highlight_end_node(struct grille_view_t* view, coord_t node) {
    SDL_LockMutex(view->mutex);
    view->endNode = node;
    SDL_UnlockMutex(view->mutex);
}

void highlight_current_node(struct grille_view_t* view, coord_t node) {
    SDL_LockMutex(view->mutex);
    view->currentNode = node;
    SDL_UnlockMutex(view->mutex);
}

void highlight_current_neighbor(struct grille_view_t* view, coord_t node) {
    SDL_LockMutex(view->mutex);
    view->neighboringNode = node;
    SDL_UnlockMutex(view->mutex);
}

void highlight_visited(struct grille_view_t* view, liste_noeud_t* visited) {
    SDL_LockMutex(view->mutex);
    view->visited = visited;
    SDL_UnlockMutex(view->mutex);
}


static SDL_Color color_scale(SDL_Color from, SDL_Color to, double ratio) {
    SDL_Color result = {
        .r = (Uint8) CLAMP_UINT8((double) from.r + (double) (to.r - from.r) * ratio),
        .g = (Uint8) CLAMP_UINT8((double) from.g + (double) (to.g - from.g) * ratio),
        .b = (Uint8) CLAMP_UINT8((double) from.b + (double) (to.b - from.b) * ratio),
        .a = 255
    };
    return result;
}

static const SDL_Color BLACK = { .r = 0, .g = 0, .b = 0, .a = 255 };
static const SDL_Color WHITE = { .r = 255, .g = 255, .b = 255, .a = 255 };
static const SDL_Color GREY = { .r = 180, .g = 180, .b = 180, .a = 150 };
static const SDL_Color RED1 = { .r = 180, .g = 10 , .b = 10, .a = 255 };
static const SDL_Color YELLOW1 = { .r = 255, .g = 255, .b = 197, .a = 255 };

static const SDL_Color BLUE = { .r = 0, .g = 0, .b = 180, .a = 255 };
static const SDL_Color GREEN = { .r = 0, .g = 180, .b = 0, .a = 255 };
static const SDL_Color RED2 = { .r = 180, .g = 0, .b = 0, .a = 255 };

static void draw_node(struct window_t* window, struct grille_view_t* view, coord_t position) {
    point_t center = coord_to_point(position);
    rect_t rect;
    mkrect_center(&rect, center, 1.0, 1.0);

    SDL_Rect realRect;
    from_real_rect(view->viewport, &realRect, rect);

    double h = get_hauteur(*(view->grille), position);
    SDL_Color color;
    if (h == INFINITY) {
        color = BLACK;
    } else {
        color = color_scale(YELLOW1, RED1, h);
    }

    render_rect_fill(window, &realRect, color);
}


static void draw_path_node(struct window_t* window, struct grille_view_t* view, coord_t node) {
    SDL_Point center;
    from_real_point(view->viewport, &center, coord_to_point(node));

    render_circle(window, center.x, center.y, NODE_RADIUS / view->viewport.scale_x, BLACK);
}

static void draw_path_edge(struct window_t* window, struct grille_view_t* view, coord_t src, coord_t dst) {
    SDL_Point srcR, dstR;
    from_real_point(view->viewport, &srcR, coord_to_point(src));
    from_real_point(view->viewport, &dstR, coord_to_point(dst));

    render_line(window, srcR.x, srcR.y, dstR.x, dstR.y, BLUE);
}

static void draw_visited_node(struct window_t* window, struct grille_view_t* view, coord_t node) {
    SDL_Point nodeR;
    from_real_point(view->viewport, &nodeR, coord_to_point(node));

    render_circle_fill(window, nodeR.x, nodeR.y, NODE_RADIUS / view->viewport.scale_x, GREY);
}

static void draw_visited_edge(struct window_t* window, struct grille_view_t* view, coord_t src, coord_t dst) {
    SDL_Point srcR, dstR;
    from_real_point(view->viewport, &srcR, coord_to_point(src));
    from_real_point(view->viewport, &dstR, coord_to_point(dst));

    render_line(window, srcR.x, srcR.y, dstR.x, dstR.y, GREY);
}

static void draw_start_node(struct window_t* window, struct grille_view_t* view, coord_t node) {
    SDL_Point nodeR;
    from_real_point(view->viewport, &nodeR, coord_to_point(node));

    render_circle_fill(window, nodeR.x, nodeR.y, NODE_RADIUS / (2.0 * view->viewport.scale_x), BLACK);
}

static void draw_end_node(struct window_t* window, struct grille_view_t* view, coord_t node) {
    SDL_Point nodeR;
    from_real_point(view->viewport, &nodeR, coord_to_point(node));

    render_circle_fill(window, nodeR.x, nodeR.y, NODE_RADIUS / (2.0 * view->viewport.scale_x), BLACK);
}

static void draw_current_node(struct window_t* window, struct grille_view_t* view, coord_t node) {
    SDL_Point nodeR;
    from_real_point(view->viewport, &nodeR, coord_to_point(node));

    render_circle_fill(window, nodeR.x, nodeR.y, NODE_RADIUS / view->viewport.scale_x, GREEN);
}

static void draw_neighbor_node(struct window_t* window, struct grille_view_t* view, coord_t node) {
    SDL_Point nodeR;
    from_real_point(view->viewport, &nodeR, coord_to_point(node));

    render_circle_fill(window, nodeR.x, nodeR.y, NODE_RADIUS / view->viewport.scale_x, RED2);
}

void grille_view_repaint(struct window_t* window, void* userdata) {
    struct grille_view_t* view = (struct grille_view_t*) userdata;

    SDL_LockMutex(view->mutex);
    for (int x = 0; x < get_largeur(*(view->grille)); x++) {
        for (int y = 0; y < get_profondeur(*(view->grille)); y++) {
            coord_t c = creer_coord(x, y);
            draw_node(window, view, c);
        }
    }
    for (int x = 0; x < get_largeur(*(view->grille)); x++) {
        for (int y = 0; y < get_profondeur(*(view->grille)); y++) {
            coord_t c = creer_coord(x, y);
            if (view->visited != NULL && contient_noeud_liste(view->visited, c)) {
                draw_visited_node(window, view, c);
                coord_t prec = precedent_noeud_liste(view->visited, c);
                if (dans_les_bornes(*(view->grille), prec)) {
                    draw_visited_edge(window, view, prec, c);
                }
            }
            if (memes_coord(c, view->startNode)) {
                draw_start_node(window, view, c);
            }
            if (memes_coord(c, view->endNode)) {
                draw_end_node(window, view, c);
            }
            if (memes_coord(c, view->currentNode)) {
                draw_current_node(window, view, c);
            }
            if (memes_coord(c, view->neighboringNode)) {
                draw_neighbor_node(window, view, c);
            }
            if (view->path != NULL && contient_noeud_liste(view->path, c)) {
                draw_path_node(window, view, c);
                coord_t prec = precedent_noeud_liste(view->path, c);
                if (contient_arrete_liste(view->path, prec, c)) {
                    draw_path_edge(window, view, prec, c);
                }
            }
        }
    }
    SDL_UnlockMutex(view->mutex);
}



