#ifndef GRILLE_FILE_H
#define GRILLE_FILE_H

#include "grille.h"

void export_grille(grille_t grille, const char* filename);
void import_grille(grille_t* grille, const char* filename);

#endif



