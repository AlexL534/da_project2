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
    Duration: 2.74e-03 seconds
    Minimum cost: 354.5

2-opt tourism:
    Duration: 8.50e-05 seconds
    Minimum cost: 2600.0

Triangular Real 1:
    Duration: 6.67e+00 seconds
    Minimum cost: 1121854.3

Triangular 25 edges:
    Duration: 1.17e-02 seconds
    Minimum cost: 364937.2

Triangular 50 edges:
    Duration: 1.32e-02 seconds
    Minimum cost: 542185.9

Triangular 75 edges:
    Duration: 1.83e-02 seconds
    Minimum cost: 626275.9

Triangular 900 edges:
    Duration: 4.92e+00 seconds
    Minimum cost: 1991369.1

2-opt 25 edges:
    Duration: 4.24e-02 seconds
    Minimum cost: 280592.3

Triangular Real 3:
    Duration: 3.74e+02 seconds
    Minimum cost: 2903860.8

Lin-Kernighan tourism:
    Duration: 8.20e-05 seconds
    Minimum cost: 2600.0

Lin-Kernighan stadiums:
    Duration: 1.41e-03 seconds
    Minimum cost: 348.6
*/



