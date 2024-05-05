#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "graph.h"
#include "parse.h"

Graph* parseToyGraph(const std::string& filename) {
    auto* graph = new Graph();
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return nullptr;
    }
    std::string line;
    getline(file,line); //ignorar primeira linha
    while(getline(file,line)) {
        string origem, destino;
        double distancia;
        string labelOrigem, labelDestino;
        bool labels = false;
        std::istringstream iss(line);
        getline(iss,origem,',');
        getline(iss,destino,',');
        iss >> distancia;
        iss.ignore();

        if (std::getline(iss, labelOrigem, ',')) {
            std::getline(iss, labelDestino, ',');
            labels = true;
        }

        if (graph->findVertex(origem) == nullptr) {
            graph->addVertex(origem);
            if (labels) graph->findVertex(origem)->setLabel(labelOrigem);
        }

        if (graph->findVertex(destino) == nullptr) {
            graph->addVertex(destino);
            if (labels) graph->findVertex(destino)->setLabel(labelDestino);
        }

        graph->addEdge(origem,destino,distancia);
        graph->addEdge(destino,origem,distancia);
    }
    return graph;
}
