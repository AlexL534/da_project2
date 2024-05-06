#include <iostream>
#include <cassert>
#include "src/parse.h"

void testShippingGraphConstruction() {
    std::string filepath = "../Datasets/Toy-Graphs/Toy-Graphs/shipping.csv";
    Graph* graph = parseToyGraph(filepath);

    if (graph == nullptr) {
        std::cerr << "Error: Failed to parse graph from file: " << filepath << std::endl;
        return;
    }

    assert(graph->getNumVertex() == 14 && "Failed: Incorrect number of vertices");

    assert(graph->getNumEdges() == 34 && "Failed: Incorrect number of edges");

    auto edge01 = graph->findEdge("0", "1");
    assert(edge01 != nullptr && edge01->getWeight() == 3.1 && "Failed: Incorrect weight for edge between vertices 0 and 1");

    auto edge910 = graph->findEdge("9", "10");
    assert(edge910 != nullptr && edge910->getWeight() == 7.0 && "Failed: Incorrect weight for edge between vertices 9 and 10");

    std::cout << "All tests passed successfully!\n";
}


void testStadiumsGraphConstruction() {
    std::string filepath = "../Datasets/Toy-Graphs/Toy-Graphs/stadiums.csv";
    Graph* graph = parseToyGraph(filepath);

    if (graph == nullptr) {
        std::cerr << "Error: Failed to parse graph from file: " << filepath << std::endl;
        return;
    }

    assert(graph->getNumVertex() == 11 && "Failed: Incorrect number of vertices");

    assert(graph->getNumEdges() == 55 && "Failed: Incorrect number of edges");

    auto edge01 = graph->findEdge("0", "1");
    assert(edge01 != nullptr && edge01->getWeight() == 4.6 && "Failed: Incorrect weight for edge between vertices 0 and 1");

    auto edge27 = graph->findEdge("2", "7");
    assert(edge27 != nullptr && edge27->getWeight() == 93.5 && "Failed: Incorrect weight for edge between vertices 2 and 7");

    auto edge710 = graph->findEdge("7", "10");
    assert(edge710 != nullptr && edge710->getWeight() == 80.9 && "Failed: Incorrect weight for edge between vertices 7 and 10");

    auto edge910 = graph->findEdge("9", "10");
    assert(edge910 != nullptr && edge910->getWeight() == 36.3 && "Failed: Incorrect weight for edge between vertices 9 and 10");

    std::cout << "All tests passed successfully!\n";
}

void testTourismGraphConstruction() {
    std::string filepath = "../Datasets/Toy-Graphs/Toy-Graphs/tourism.csv";
    Graph* graph = parseToyGraph(filepath);

    if (graph == nullptr) {
        std::cerr << "Error: Failed to parse graph from file: " << filepath << std::endl;
        return;
    }

    assert(graph->getNumVertex() == 5 && "Failed: Incorrect number of vertices");

    assert(graph->getNumEdges() == 10 && "Failed: Incorrect number of edges");

    auto vertex0 = graph->findVertex("0");
    assert(vertex0 != nullptr && vertex0->getLabel() == "carmo" && "Failed: Incorrect label for vertex 0");

    auto vertex3 = graph->findVertex("3");
    assert(vertex3 != nullptr && vertex3->getLabel() == "clerigos" && "Failed: Incorrect label for vertex 3");

    auto edge13 = graph->findEdge("1", "3");
    assert(edge13 != nullptr && edge13->getWeight() == 950 && "Failed: Incorrect weight for edge between vertices 1 and 3");

    std::cout << "All tests passed successfully!\n";
}

void runAllToyParseTests() {
    // Test for shipping.csv
    std::cout << "\nRunning test for shipping.csv...\n";
    testShippingGraphConstruction();

    // Test for stadiums.csv
    std::cout << "\nRunning test for stadiums.csv...\n";
    testStadiumsGraphConstruction();

    // Test for tourism.csv
    std::cout << "\nRunning test for tourism.csv...\n";
    testTourismGraphConstruction();
}

void testExtra25GraphConstruction() {
    std::string nodes_filepath = "../Datasets/Extra_Fully_Connected_Graphs/Extra_Fully_Connected_Graphs/nodes.csv";
    std::string edges_filepath = "../Datasets/Extra_Fully_Connected_Graphs/Extra_Fully_Connected_Graphs/edges_25.csv";

    Graph* graph = parseExtraFullyConnectedGraph(edges_filepath, nodes_filepath);

    if (graph == nullptr) {
        std::cerr << "Error: Failed to parse graph from files"<< std::endl;
        return;
    }

    assert(graph->getNumEdges() == 300 && "Failed: Incorrect number of edges");

    auto vertex0 = graph->findVertex("0");
    assert(vertex0 != nullptr && vertex0->getLongitude() == -43.19448871895534 && vertex0->getLatitude() == -22.983571669284053 && "Failed: Incorrect coordinates for vertex 0");

    auto vertex6474 = graph->findVertex("6474");
    assert(vertex6474 != nullptr && vertex6474->getLongitude() == -43.41330766502478 && vertex6474->getLatitude() == -22.86965572303136 && "Failed: Incorrect coordinates for vertex 0");

    auto edge13 = graph->findEdge("1", "3");
    assert(edge13 != nullptr && edge13->getWeight() == 39557.0 && "Failed: Incorrect weight for edge between vertices 1 and 3");

    auto edge35 = graph->findEdge("3", "5");
    assert(edge35 != nullptr && edge35->getWeight() == 48857.7 && "Failed: Incorrect weight for edge between vertices 3 and 5");

    std::cout << "All tests passed successfully!\n";
}