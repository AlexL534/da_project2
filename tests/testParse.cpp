#include <iostream>
#include <cassert>

void testTourismGraphConstruction() {
    std::string filepath = "../Datasets/Toy-Graphs/Toy-Graphs/tourism.csv";
    Graph* graph = parseToyGraph(filepath);

    if (graph == nullptr) {
        std::cerr << "Error: Failed to parse graph from file: " << filepath << std::endl;
        return;
    }

    // Check the number of vertices
    assert(graph->getNumVertex() == 5 && "Failed: Incorrect number of vertices");

    // Check the number of edges
    assert(graph->getNumEdges() == 10 && "Failed: Incorrect number of edges");

    // Check the label of vertex "0"
    auto vertex0 = graph->findVertex("0");
    assert(vertex0 != nullptr && vertex0->getLabel() == "carmo" && "Failed: Incorrect label for vertex 0");

    // Check the label of vertex "3"
    auto vertex3 = graph->findVertex("3");
    assert(vertex3 != nullptr && vertex3->getLabel() == "clerigos" && "Failed: Incorrect label for vertex 3");

    // Check the weight of the edge between vertices "1" and "3"
    auto edge13 = graph->findEdge("1", "3");
    assert(edge13 != nullptr && edge13->getWeight() == 950 && "Failed: Incorrect weight for edge between vertices 1 and 3");

    std::cout << "All tests passed successfully!\n";
}