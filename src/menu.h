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
                    std::cout << "Choose the approach:\n1. Backtracking Algorithm\n2. Triangular Approximation Algorithm\n3. Other heuristics\n4. Real world approach\n5. Go back\n";
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
                            a:
                            std::string strt;
                            std::string c;
                            std::string c1;
                            std::string iterations;
                            vector<Vertex*> solution;
                            double initialTemp = 10.0;
                            double finalTemp = 0.001;
                            double alpha = 0.95;
                            double totalCost;
                            bool a = true;
                            int number = 0;
                            Graph* g = graph;
                            cout << "Select the starting vertex label: \n";
                            if(graphType == 2){
                            }
                            cout << "Option range: 0 to " << graph->getVertexMap().size() - 1 << " "; cout << endl;
                            cin >> strt;
                            auto start = std::chrono::high_resolution_clock::now();
                            double minCost = NNTSP(graph, strt, totalCost, solution);
                            auto end = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                            std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration.count()) * 1e-6 << " seconds" << std::endl;
                            std::cout << "Minimum cost: " << std::fixed << std::setprecision(1) << minCost << std::endl;
                            cout << "\nDo you want to run Simulated annealing on top of the solution? (y/n)\n";
                            cin >> c1;
                            if(c1 == "y" || c1=="Y" || c1=="yes" || c1=="Yes" || c1=="YES"){
                                cout << "\nEnter the number of iterations: ";
                                cin >> iterations;
                                auto start1 = std::chrono::high_resolution_clock::now();
                                double Cost = SimulatedAnnealing(g, solution, totalCost, initialTemp, finalTemp, alpha, stoi(iterations));
                                auto end1 = std::chrono::high_resolution_clock::now();
                                auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);
                                std::cout << "Duration: " << std::scientific << std::setprecision(2) << static_cast<double>(duration1.count()) * 1e-6 << " seconds" << std::endl;
                                std::cout << "Minimum cost: " << std::fixed << std::setprecision(1) << Cost << std::endl;
                            }
                            cout << "\nTry another vertex? (y/n)\n";
                            cin >> c;
                            if(c == "y" || c=="Y" || c=="yes" || c=="Yes" || c=="YES") {graph = g; totalCost = 0; goto a;}

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