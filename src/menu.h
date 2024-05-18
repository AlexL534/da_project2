#ifndef MENU_H
#define MENU_H

#include "iostream"
#include "parse.h"
#include "Actions.h"
#include <ctime>
#include <chrono>
#include <iomanip>

void menu() {
    while (true) {
        int graphType;
        int graphChoice;
        int approachChoice;
        int connectionType;
        Graph* graph = nullptr;
        bool goBackSelected = false;

        while (true) {
            std::cout << "Choose the type of graph:\n1. Toy-Graphs\n2. Extra-Fully-Connected-Graphs\n3. Real-world Graphs\n4. Exit\n";
            std::cin >> graphType;

            if (graphType == 4) exit(0);

            switch(graphType) {
                case 1: {
                    std::cout << "Choose the connection type:\n1. Not Fully Connected\n2. Fully Connected\n3. Go back\n";
                    std::cin >> connectionType;

                    if (connectionType == 3) {
                        goBackSelected = true;
                        break;
                    }

                    switch(connectionType) {
                        case 1: {
                            std::vector<std::string> notFullyConnectedGraphs = {"shipping"};
                            std::cout << "Choose the graph:\n1. shipping\n2. Go back\n";
                            std::cin >> graphChoice;

                            if (graphChoice == 2) {
                                goBackSelected = true;
                                break;
                            }

                            graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/" + notFullyConnectedGraphs[graphChoice - 1] + ".csv");
                            break;
                        }
                        case 2: {
                            std::vector<std::string> fullyConnectedGraphs = {"stadiums", "tourism"};
                            std::cout << "Choose the graph:\n1. stadiums\n2. tourism\n3. Go back\n";
                            std::cin >> graphChoice;

                            if (graphChoice == 3) {
                                goBackSelected = true;
                                break;
                            }

                            graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/" + fullyConnectedGraphs[graphChoice - 1] + ".csv");
                            break;
                        }
                        default:
                            std::cout << "Invalid option. Please try again.\n";
                            std::cin.clear(); // Clear the error state
                            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
                            continue;
                    }
                    break;
                }
                case 2: {
                    std::vector<std::string> extraGraphs = {"edges_25", "edges_50", "edges_75", "edges_100", "edges_200", "edges_300", "edges_400", "edges_500", "edges_600", "edges_700", "edges_800", "edges_900"};
                    std::cout << "Choose the graph:\n1. edges_25\n2. edges_50\n3. edges_75\n4. edges_100\n5. edges_200\n6. edges_300\n7. edges_400\n8. edges_500\n9. edges_600\n10. edges_700\n11. edges_800\n12. edges_900\n";
                    std::cin >> graphChoice;
                    graph = parseExtraFullyConnectedGraph("../Datasets/Extra_Fully_Connected_Graphs/Extra_Fully_Connected_Graphs/" + extraGraphs[graphChoice - 1] + ".csv", "../Datasets/Extra_Fully_Connected_Graphs/Extra_Fully_Connected_Graphs/nodes.csv");
                    break;
                }
                case 3: {
                    std::vector<std::string> realWorldGraphs = {"graph1", "graph2", "graph3"};
                    std::cout << "Choose the graph:\n1. graph1\n2. graph2\n3. graph3\n";
                    std::cin >> graphChoice;
                    graph = parseRealWorldGraph("../Datasets/Real-world Graphs/Real-world Graphs/" + realWorldGraphs[graphChoice - 1] + "/edges.csv", "../Datasets/Real-world Graphs/Real-world Graphs/" + realWorldGraphs[graphChoice - 1] + "/nodes.csv");
                    break;
                }
                default:
                    std::cout << "Invalid option. Please try again.\n";
                    std::cin.clear(); // Clear the error state
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
                    continue;
            }

            if (graph == nullptr) {
                if (!goBackSelected) std::cout << "Error: Unable to load graph.\n";
            } else {
                break;
            }
        }

        while (true) {
            std::cout << "Choose the approach:\n1. Backtracking Algorithm\n2. Triangular Approximation Algorithm\n3. 2-opt\n4. Go back\n";
            std::cin >> approachChoice;

            if (approachChoice == 4) {
                goBackSelected = true;
                break;
            }

            switch(approachChoice) {
                case 1:
                {
                    auto start = std::chrono::high_resolution_clock::now();
                    double minCost = TSPBacktracking(graph);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                    std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration.count()) * 1e-6 << " seconds" << std::endl;
                    std::cout << "Minimum cost: " << std::fixed << std::setprecision(1) << minCost << std::endl;
                    return;
                }
                case 2:
                {
                    if (graphType == 1 && connectionType == 1) {
                        std::cout << "This algorithm does not work with this graph (not fully connected and no coordinates given)\n\n";
                        continue;
                    }

                    auto start = std::chrono::high_resolution_clock::now();
                    double minCost = TSPTriangularApproximation(graph);
                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                    std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration.count()) * 1e-6 << " seconds" << std::endl;
                    std::cout << "Minimum cost: " << std::fixed << std::setprecision(1) << minCost << std::endl;
                    return;
                }
                case 3:
                {
                    std::cout << "This algorithm had not been implemented when I commited this)\n\n";
                    auto start = std::chrono::high_resolution_clock::now();
                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                    std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration.count()) * 1e-6 << " seconds" << std::endl;
                    return;
                }
                default:
                    std::cout << "Invalid option. Please try again.\n";
                    std::cin.clear(); // Clear the error state
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
                    continue;
            }
        }
    }
}

#endif // MENU_H