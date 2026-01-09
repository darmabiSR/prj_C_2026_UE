#include "animation.h"
#include <SDL2/SDL_timer.h>

struct animation_conf_t {
    struct grille_view_t* view;
    struct app_t* app;
    bool enabled;
    Uint32 delay;
};

static struct animation_conf_t config;


void setup_animation(struct app_t* app, struct grille_view_t* view) {
    config.app = app;
    config.view = view;
}

void disable_animation() {
    config.enabled = false;
}

void enable_animation() {
    config.enabled = true;
}

void set_animation_delay(unsigned int delay) {
    config.delay = (Uint32) delay;
}

void animate_source(coord_t node) {
    if (!config.enabled) return;
    highlight_start_node(config.view, node);
    refresh_app(config.app);
    SDL_Delay(config.delay);
}

void animate_destination(coord_t node) {
    if (!config.enabled) return;
    highlight_end_node(config.view, node);
    refresh_app(config.app);
    SDL_Delay(config.delay);
}

void animate_visited(liste_noeud_t* visited) {
    if (!config.enabled) return;
    highlight_visited(config.view, visited);
    refresh_app(config.app);
    SDL_Delay(config.delay);
}

void animate_visiting(coord_t node) {
    if (!config.enabled) return;
    highlight_current_node(config.view, node);
    refresh_app(config.app);
    SDL_Delay(config.delay);
}

void animate_neighbor(coord_t neighbor) {
    if (!config.enabled) return;
    highlight_current_neighbor(config.view, neighbor);
    refresh_app(config.app);
    SDL_Delay(config.delay);
}

void animate_path(liste_noeud_t* path) {
    if (!config.enabled) return;
    highlight_path(config.view, path);
    refresh_app(config.app);
    SDL_Delay(config.delay);
}

void clear_source() {
    animate_source(creer_coord(-1, -1));
}

void clear_destination() {
    animate_destination(creer_coord(-1, -1));
}

void clear_visited() {
    animate_visited(NULL);
}

void clear_visiting() {
    animate_visiting(creer_coord(-1, -1));
}

void clear_neighbor() {
    animate_neighbor(creer_coord(-1, -1));
}

void clear_path() {
    animate_path(NULL);
}

