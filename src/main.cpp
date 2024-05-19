#include "tests/testParse.cpp"
#include "tests/testTSPBacktracking.cpp"
#include "menu.h"

int main() {
    menu();
    return 0;
}

/*
Vou guardar aqui alguns resultados

Shipping:
    Backtracking:
        Duration: 2.4e-02 seconds
        Minimum cost: 86.7

Stadiums:
    Backtracking:
        Duration: 1.16e-02 seconds
        Minimum cost: 341.0
    Triangular:
        Duration: 2.80e-04 seconds
        Minimum cost: 398.1
    2-opt:
        Duration: 2.74e-03 seconds
        Minimum cost: 354.5
    Lin-Kernighan:
        Duration: 1.41e-03 seconds
        Minimum cost: 348.6

Tourism:
    Backtracking:
        Duration: 7.50e-05 seconds
        Minimum cost: 2600.0
    Triangular:
        Duration: 8.30e-05 seconds
        Minimum cost: 2600.0

25 edges:
    Backtracking:
        Duration: 2.17e+03 seconds
        Minimum cost: 228387.7
    Triangular:
        Duration: 1.17e-02 seconds
        Minimum cost: 364937.2

50 edges:
    Triangular:
        Duration: 1.32e-02 seconds
        Minimum cost: 542185.9

75 edges:
    Triangular:
        Duration: 1.83e-02 seconds
        Minimum cost: 626275.9

900 edges:
    Triangular:
        Duration: 4.92e+00 seconds
        Minimum cost: 1991369.1

Real 1:
    Triangular:
        Duration: 6.67e+00 seconds
        Minimum cost: 1121854.3

Real 3:
    Triangular:
        Duration: 3.74e+02 seconds
        Minimum cost: 2903860.8
*/



