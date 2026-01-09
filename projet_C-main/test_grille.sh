#!/bin/bash
# Petit script bash qui test la conformance des fichiers rendus
#
# /!\ NE PAS MODIFIER /!\

logfile="checkerror.log"
err=0

check_fichier () {
    if [ ! -f "$1" ]; then
        echo "Je ne trouve pas le fichier $1 ! Vérifiez que vous êtes bien dans le bon répertoire..."
        echo "Fichier $1 absent" >>$logfile
        err=1
        return 1
    fi

    echo "Vérification de la syntaxe pour $1 :" >>$logfile
    echo "gcc -std=c99 -c -Wall -Wextra -pedantic -Werror -fsyntax-only -x c $1" >>$logfile 
    gcc -std=c99 -c -Wall -Wextra -pedantic -Werror -fsyntax-only -x c "$1" >>$logfile 2>&1

    if [ $? -ne 0 ]; then
        echo "Je n'arrive pas à compiler le fichier $1... Veuillez corriger les erreurs et recommencer"
        err=1
    fi
}

check_garde () {
    include="$1"
    anchor="$2"
    tmpfile="__tmp_${include}"

    if [ -f "${tmpfile}" ]; then
        rm "${tmpfile}"
    fi

    # Ajoute une constante statique
    sed "/$anchor/a\
        static const int x = 1;" ${include} > ${tmpfile}
 
    read -r -d '' RESULT <<END_DOC
#include "${tmpfile}"
#include "${tmpfile}"

void test() {}
END_DOC

    echo "" >>$logfile
    echo "[le header ${include} est correctement gardé]" >>$logfile
    echo "==code" >>$logfile
    echo "$code" >>$logfile
    echo "==endcode" >>$logfile
    echo "$RESULT" | gcc -c -fsyntax-only -x c - >>$logfile 2>&1

    rm "${tmpfile}"

    if [[ ($neg -eq 0 && $? -ne 0) || ($neg -eq 1 && $? -eq 0) ]]; then
        echo "Erreur : le header ${include} n'est pas correctement gardé !"
        err=1
        return 1
    fi

    return 0
}

check () {
    name=""
    include=""
    code=""
    errmsg=""
    neg=0

    if [[ $1 == "-n" ]]; then
        name="$2"
        include="$3"
        code="$4"
        errmsg="$5"
        neg=1
    else
        name="$1"
        include="$2"
        code="$3"
        errmsg="$4"
    fi

    read -r -d '' RESULT <<END_DOC
#include "${include}"

void test() {
${code}
}
END_DOC

    echo "" >>$logfile
    echo "[${name}]" >>$logfile
    echo "==code" >>$logfile
    echo "$code" >>$logfile
    echo "==endcode" >>$logfile
    echo "$RESULT" | gcc -c -Wall -Wextra -pedantic -Werror -fsyntax-only -x c - >>$logfile 2>&1

    if [[ ($neg -eq 0 && $? -ne 0) || ($neg -eq 1 && $? -eq 0) ]]; then
        echo "Erreur : ${errmsg}"
        err=1
        return 1
    fi

    return 0
}

if [ -f $logfile ]; then
    rm $logfile
fi

echo "Vérification des fichiers..."

check_fichier "grille.h"
check_fichier "grille.c"

if [ $err -ne 0 ]; then
    echo "Les erreurs des fichiers testés empêchent de poursuivre le test..."
    echo "Consultez le fichier ${logfile} pour voir les messages d'erreur obtenu"
    exit -1
fi

gerr=0

echo "Compilation..."

gcc -c test_grille.c -o test_grille.o -O0 && \
gcc -c grille.c -o grille.o -O0 && \
gcc -c coord.c -o coord.o -O0 && \
gcc test_grille.o grille.o coord.o -o test_grille -L. -lt -lm

if [ $? -ne 0 ]; then
    echo "Impossible de compiler le test :("
    gerr=2
fi

if [ $gerr -eq 0 ]; then
    echo "Check ok."
else
    exit -2
fi

echo "Lancement des tests..."

./test_grille





