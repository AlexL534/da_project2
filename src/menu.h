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
        Graph* graph = nullptr;

        while (true) {
            std::cout << "Choose the type of graph:\n1. Toy-Graphs\n2. Extra-Fully-Connected-Graphs\n3. Real-world Graphs\n4. Exit\n";
            std::cin >> graphType;

            if (graphType == 4) return;

            switch(graphType) {
                case 1: {
                    std::vector<std::string> toyGraphs = {"shipping", "stadiums", "tourism"};
                    std::cout << "Choose the graph:\n1. shipping\n2. stadiums\n3. tourism\n";
                    std::cin >> graphChoice;
                    graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/" + toyGraphs[graphChoice - 1] + ".csv");
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
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    continue;
            }

            if (graph == nullptr) {
                std::cout << "Error: Unable to load graph.\n";
            } else {
                bool chooseApproach = true;
                while (chooseApproach) {
                    int approachChoice;
                    std::cout << "Choose the approach:\n1. Backtracking Algorithm\n2. Triangular Approximation Algorithm\n3. Other heuristics\n4. Not implemented\n5. Go back\n";
                    std::cin >> approachChoice;

                    if (approachChoice == 5) {
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
                            break;
                        }
                        case 2:
                        {
                            if (graphType == 1 && graphChoice == 1) {
                                std::cout << "This algorithm does not work with this graph (not fully connected and no coordinates given)\n\n";
                                continue;
                            }
                            auto start = std::chrono::high_resolution_clock::now();
                            double minCost = TSPTriangularApproximation(graph);
                            auto end = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                            std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration.count()) * 1e-6 << " seconds" << std::endl;
                            std::cout << "Minimum cost: " << std::fixed << std::setprecision(1) << minCost << std::endl;
                            break;
                        }
                        case 3:
                        {
                            if (graphType == 1 && graphChoice == 1) {
                                std::cout << "This algorithm does not work with this graph (not fully connected and no coordinates given)\n\n";
                                continue;
                            }

                            auto start = std::chrono::high_resolution_clock::now();
                            double minCost = TSPChristofides(graph);
                            auto end = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                            std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration.count()) * 1e-6 << " seconds" << std::endl;
                            std::cout << "Minimum cost: " << std::fixed << std::setprecision(1) << minCost << std::endl;
                            break;
                        }
                        case 4:
                        {
                            auto start = std::chrono::high_resolution_clock::now();
                            auto end = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                            std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration.count()) * 1e-6 << " seconds" << std::endl;
                            break;
                        }
                        default:
                            std::cout << "Invalid option. Please try again.\n";
                            std::cin.clear();
                            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            continue;
                    }

                    std::cout << "Do you want to continue using the program or Exit?\n1. Continue\n2. Exit\n";
                    int continueChoice;
                    std::cin >> continueChoice;
                    if (continueChoice == 2) {
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        return;
                    }
                    if (continueChoice == 1) {
                        chooseApproach = false;
                    }
                }
            }
        }
    }
}
#endif // MENU_H