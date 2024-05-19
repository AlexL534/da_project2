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
        Duration: 2.60e-04 seconds
        Minimum cost: 398.1
    Christofides:
        Duration: 1.6e-04 seconds  1.9
        Minimum cost: 391.4

Tourism:
    Backtracking:
        Duration: 7.50e-05 seconds
        Minimum cost: 2600.0
    Triangular:
        Duration: 7.40e-05 seconds
        Minimum cost: 2600.0
    Christofides:
        Duration: 7.20e-05 seconds
        Minimum cost: 2600.0

25 edges:
    Backtracking:
        Duration: 2.17e+03 seconds
        Minimum cost: 228387.7
    Triangular:
        Duration: 1.17e-02 seconds
        Minimum cost: 364937.2
    Christofides:
        Duration: 6.52e-03 seconds
        Minimum cost: 362891.5

50 edges:
    Triangular:
        Duration: 1.32e-02 seconds
        Minimum cost: 542185.9
    Christofides:
        Duration: 8.25e-03 seconds
        Minimum cost: 571948.2

75 edges:
    Triangular:
        Duration: 1.83e-02 seconds
        Minimum cost: 626275.9
    Christofides:
        Duration: 9.73e-03 seconds
        Minimum cost: 646110.8

500 edges:
    Triangular:
        Duration: 1.02e+00 seconds
        Minimum cost: 1422316.5
    Christofides:
        Duration: 2.32e-01 seconds
        Minimum cost: 1501943.1

900 edges:
    Triangular:
        Duration: 4.92e+00 seconds
        Minimum cost: 1991369.1
    Christofides:
        Duration: 5.78e+00 seconds
        Minimum cost: 2096580.6

Real 1:
    Triangular:
        Duration: 6.67e+00 seconds
        Minimum cost: 1121854.3
    Christofides:
        Duration: 1.20e+00 seconds
        Minimum cost: 1115162.9
Real 2:
    Triangular:
        Duration: 9.22e+01 seconds
        Minimum cost: 1871620.3
    Christofides:
        Duration: 1.501e+01 seconds
        Minimum cost: 2321754.4

Real 3:
    Triangular:
        Duration: 3.74e+02 seconds
        Minimum cost: 2903860.8
    Christofides:
        Duration: 4.60e+01 seconds
        Minimum cost: 3613985.9
*/



