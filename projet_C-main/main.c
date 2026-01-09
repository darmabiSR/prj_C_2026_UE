#define _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL.h>
#ifdef _DEBUG // En mode debug on affiche le PID (pour kill plus facilement)
#include <unistd.h>
#endif // _DEBUG
#include <time.h>
#include <string.h>
#include "util.h"
#include "grille.h"
#include "app.h"
#include <math.h>
#include "grille_file.h"
#include "liste_noeud.h"
#include "dijkstra.h"
#include "animation.h"
#include "creer_grille.h"

struct dijkstra_conf_t {
    grille_t* grille;
    coord_t source,
            destination;
    liste_noeud_t* chemin;
    float seuil;
    float resultat;
    double temps;
};

static int perform_dijkstra(struct app_t* app, void* userdata) {
    (void) app;
    struct dijkstra_conf_t* conf = (struct dijkstra_conf_t*) userdata;

    if (conf->chemin != NULL) {
        detruire_liste(&(conf->chemin));
    }

    clock_t begin, end;
    printf("Début du calcul du chemin le plus court...\n");
    begin = clock();
    conf->resultat = dijkstra(
            *(conf->grille),
            conf->source, conf->destination,
            conf->seuil,
            &(conf->chemin)
    );
    end = clock();

    conf->temps = ((double)(end - begin) / CLOCKS_PER_SEC);

    return 0;
}

static int show_chemin(struct app_t* app, void* userdata) {
    struct dijkstra_conf_t* conf = (struct dijkstra_conf_t*) userdata;
    highlight_path_app(app, conf->chemin);
    printf("Coût minimal : %f\nTemps de calcul : %lf ms\n", conf->resultat, conf->temps * 1.0e3);
    return 0;
}

/**
 * Le programme principal. Configure l'application, affiche la fenêtre et gère
 * les événements (clavier, souris...).
 * @param argc nombre d'arguments de la ligne de commande
 * @param argv arguments de la ligne de commande
 */
int main(int argc, char* argv[]) {
    // On affiche le PID
    MLog("pid=%d\n", getpid());
    char* file;

    if (argc < 2) {
        file = NULL;
    } else {
        if (strcmp(argv[1], "-a") == 0) {
            enable_animation();
            if (argc < 3) {
                file = NULL;
            } else {
                file = argv[2];
            }
        } else {
            disable_animation();
            file = argv[1];
        }
    }

    // Récupérer la grille
    grille_t grille = remplir_grille(file);

    coord_t debut = inferieur_gauche(grille);
    coord_t but = superieur_droit(grille);

    /************** Lancement du programme **************/
    // Activation de la librairie graphique
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    // Création de l'application
    int width = 1280;
    int height = 960;
    struct app_t* app = setup_app(width, height, &grille);

    // Lancement
    launch_app(app);

    /************** Tâche parallèle : calcul de plus cours chemin **************/
    struct dijkstra_conf_t conf;
    conf.grille = &grille;
    conf.source = debut;
    conf.destination = but;
    conf.seuil = 1.0;
    conf.chemin = NULL;
    exec_task_app(app, perform_dijkstra, show_chemin, &conf, true);

    /************** Configuration des animations **************/
    set_animation_delay(50 /* ms */);

    /************** Boucle principale : gestion des événements **************/
    SDL_Event event;
    bool quit = false;

    while (!quit) { // Tant que pas quitté
        // On attend un événement
        SDL_WaitEvent(&event);

        if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && (event.key.keysym.sym == SDLK_q || event.key.keysym.sym == SDLK_ESCAPE))) { // croix/boutin escape/boutin Q => quitter
            quit = true;
        } else if (event.type == SDL_MOUSEMOTION) { // la souris bouge => mise à jour de la croix/curseur
            move_cross_app(app, event.motion.x, event.motion.y);
        } else if (event.type == SDL_MOUSEBUTTONDOWN) { // click => zoom/dé-zoom
            // NOP
        } else if (event.type == SDL_KEYDOWN) { 
            // NOP
        }
    }

    /************** Fin du programme et nettoyage **************/
    terminate_app(&app);

    if (conf.chemin != NULL) {
        free(conf.chemin);
    }

    detruire_grille(grille);

    MLog("Terminate TTF");
    TTF_Quit();

    MLog("Terminate SDL");
    SDL_Quit();

    MLog("End");
    return 0;
}



