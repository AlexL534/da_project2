#include <iostream>
#include <vector>
#include <climits>
#include "src/Actions.h"
#include "src/parse.h"

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

        // Test with shipping graph
        graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/shipping.csv");
        min_path_cost = TSPBacktracking(graph);
        expected_min_cost = 86.7; // Expected minimum cost for shipping graph
        if (std::abs(min_path_cost - expected_min_cost) > 1e-6) { // Allow for a tolerance of 1e-6
            std::cout << "Test failed: Expected minimum cost for shipping graph = " << expected_min_cost
                      << ", Actual minimum cost = " << min_path_cost << std::endl;
        } else {
            std::cout << "Test passed: Minimum cost for shipping graph matches the expected value." << std::endl;
        }

        // Test with stadiums graph
        graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/stadiums.csv");
        min_path_cost = TSPBacktracking(graph);
        expected_min_cost = 341.0; // Expected minimum cost for stadiums graph
        if (std::abs(min_path_cost - expected_min_cost) > 1e-6) { // Allow for a tolerance of 1e-6
            std::cout << "Test failed: Expected minimum cost for stadiums graph = " << expected_min_cost
                      << ", Actual minimum cost = " << min_path_cost << std::endl;
        } else {
            std::cout << "Test passed: Minimum cost for stadiums graph matches the expected value." << std::endl;
        }

        // Test with tourism graph
        graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/tourism.csv");
        min_path_cost = TSPBacktracking(graph);
        expected_min_cost = 2600; // Expected minimum cost for tourism graph
        if (std::abs(min_path_cost - expected_min_cost) > 1e-6) { // Allow for a tolerance of 1e-6
            std::cout << "Test failed: Expected minimum cost for tourism graph = " << expected_min_cost
                      << ", Actual minimum cost = " << min_path_cost << std::endl;
        } else {
            std::cout << "Test passed: Minimum cost for tourism graph matches the expected value." << std::endl;
        }

        delete graph;
    }
}