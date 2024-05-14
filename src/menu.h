#ifndef MENU_H
#define MENU_H

#include "iostream"
#include "parse.h"
#include "Actions.h"

void menu() {
    int graphType;
    int graphChoice;
    int approachChoice;
    Graph* graph = nullptr;

    while (true) {
        std::cout << "Choose the type of graph:\n1. Toy-Graphs\n2. Extra-Fully-Connected-Graphs\n3. Real-world Graphs\n";
        std::cin >> graphType;

        switch(graphType) {
            case 1: {
                int connectionType;
                std::cout << "Choose the connection type:\n1. Not Fully Connected\n2. Fully Connected\n";
                std::cin >> connectionType;
                switch(connectionType) {
                    case 1: {
                        std::vector<std::string> notFullyConnectedGraphs = {"shipping"};
                        std::cout << "Choose the graph:\n1. shipping\n";
                        std::cin >> graphChoice;
                        graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/" + notFullyConnectedGraphs[graphChoice - 1] + ".csv");
                        break;
                    }
                    case 2: {
                        std::vector<std::string> fullyConnectedGraphs = {"stadiums", "tourism"};
                        std::cout << "Choose the graph:\n1. stadiums\n2. tourism\n";
                        std::cin >> graphChoice;
                        graph = parseToyGraph("../Datasets/Toy-Graphs/Toy-Graphs/" + fullyConnectedGraphs[graphChoice - 1] + ".csv");
                        break;
                    }
                    default:
                        std::cout << "Invalid option. Please try again.\n";
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
                continue;
        }

        if (graph == nullptr) {
            std::cout << "Error: Unable to load graph.\n";
        } else {
            break;
        }
    }

    while (true) {
        std::cout << "Choose the approach:\n1. Backtracking Algorithm\n";
        std::cin >> approachChoice;
        switch(approachChoice) {
            case 1:
            {
                double minCost = TSPBacktracking(graph);
                std::cout << "Minimum cost: " << minCost << std::endl;
                return;
            }
            default:
                std::cout << "Invalid option. Please try again.\n";
                continue;
        }
    }
}

#endif // MENU_H
