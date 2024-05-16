#include <iostream>
#include "tests/testParse.cpp"
#include "tests/testTSPBacktracking.cpp"
#include "parse.h"
#include "menu.h"

int main() {
    menu();
    return 0;
}

/*
Vou guardar aqui alguns resultados
Backtracking shipping:
    Duration: 2.4e-02 seconds
    Minimum cost: 86.7

Backtracking stadiums:
    Duration: 1.16e-02 seconds
    Minimum cost: 341.0

Backtracking tourism:
    Duration: 7.50e-05 seconds
    Minimum cost: 2600.0

Triangular stadiums:
    Duration: 2.80e-04 seconds
    Minimum cost: 398.1

Triangular tourism:
    Duration: 8.30e-05 seconds
    Minimum cost: 2600.0

2-opt stadiums:
    Duration: 3.07e-03 seconds
    Minimum cost: 354.5

2-opt tourism:
    Duration: 1.24e-04 seconds
    Minimum cost: 2600.0

Triangular Real 1:
    Duration: 6.67e+00 seconds
    Minimum cost: 1121854.3
*/