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
lines to measure time:
    clock_t startTime = clock();
    clock_t endTime = clock();
    double elapsedTime = double(endTime - startTime) / CLOCKS_PER_SEC;
    std::cout << "Elapsed time: " << elapsedTime << " seconds" << std::endl;

    ou este para microsegundos
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Duration: " << duration.count() << " microseconds" << std::endl;
 */