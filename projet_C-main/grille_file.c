#include "grille_file.h"
#include <stdlib.h>
#include <stdio.h>

void export_grille(grille_t grille, const char* filename) {
    FILE* file = fopen(filename, "wb");
    if (file == NULL) {
        perror("Impossible d'accéder au fichier");
        exit(-1);
    }
    int largeur = get_largeur(grille);
    int profondeur = get_profondeur(grille);
    float val;
    fwrite(&largeur, sizeof(int), 1, file);
    fwrite(&profondeur, sizeof(int), 1, file);
    for (int x = 0; x < largeur; x++) {
        for (int y = 0; y < profondeur; y++) {
            coord_t c = creer_coord(x, y);
            val = get_hauteur(grille, c);
            fwrite(&val, sizeof(float), 1, file);
        }
    }
    fclose(file);
}

void import_grille(grille_t* grille, const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (file == NULL) {
        perror("Impossible de lire le fichier");
        exit(-1);
    }
    int largeur, profondeur;
    fread(&largeur, sizeof(int), 1, file);
    fread(&profondeur, sizeof(int), 1, file);
    *grille = creer_grille(largeur, profondeur);
    for (int x = 0; x < largeur; x++) {
        for (int y = 0; y < profondeur; y++) {
            coord_t c = creer_coord(x, y);
            float val;
            fread(&val, sizeof(float), 1, file);
            set_hauteur(*grille, c, val);
        }
    }
}


