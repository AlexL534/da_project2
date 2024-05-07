#include <iostream>
#include <vector>
#include <climits>
#include "src/Actions.h"

namespace {

    void testTSPBacktracking() {
        auto graph = new Graph();

        graph->addVertex("0");
        graph->addVertex("1");
        graph->addVertex("2");
        graph->addVertex("3");

        graph->addEdge("0", "1", 10);
        graph->addEdge("0", "2", 15);
        graph->addEdge("0", "3", 20);
        graph->addEdge("1", "2", 35);
        graph->addEdge("1", "3", 25);
        graph->addEdge("2", "3", 30);
        graph->addEdge("1", "0", 10); //from this one down it's the reverse
        graph->addEdge("2", "0", 15);
        graph->addEdge("3", "0", 20);
        graph->addEdge("2", "1", 35);
        graph->addEdge("3", "1", 25);
        graph->addEdge("3", "2", 30);


        double min_path_cost = TSPBacktracking(graph);

        double expected_min_cost = 80;

        if (min_path_cost != expected_min_cost) {
            std::cout << "Test failed: Expected minimum cost = " << expected_min_cost
                      << ", Actual minimum cost = " << min_path_cost << std::endl;
        } else {
            std::cout << "Test passed: Minimum cost matches the expected value." << std::endl;
        }

        delete graph;
    }
}