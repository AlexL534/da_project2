#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "graph.h"
#include "parse.h"

Graph* parseToyGraph(const std::string& filepath) {
    auto* graph = new Graph();
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filepath << std::endl;
        return nullptr;
    }
    std::string line;
    getline(file,line); // Ignore first line (header)
    while(getline(file,line)) {
        string origem, destino, distancia_str;
        double distancia;
        string labelOrigem, labelDestino;
        bool labels = false;
        std::istringstream iss(line);
        getline(iss,origem,',');
        getline(iss,destino,',');
        getline(iss,distancia_str,',');
        distancia = std::stod(distancia_str);

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

Graph* parseExtraFullyConnectedGraph(const std::string& edges_filepath, const std::string& nodes_filepath) {
    auto* graph = new Graph();

    // Parse nodes
    std::ifstream nodes_file(nodes_filepath);
    if (!nodes_file.is_open()) {
        std::cerr << "Error: Unable to open nodes file " << nodes_filepath << std::endl;
        return nullptr;
    }
    std::string nodes_line;
    getline(nodes_file, nodes_line); // Ignore first line (header)
    while(getline(nodes_file, nodes_line)) {
        std::istringstream nodes_iss(nodes_line);
        std::string id_str, longitude_str, latitude_str;
        getline(nodes_iss, id_str, ',');
        getline(nodes_iss, longitude_str, ',');
        getline(nodes_iss, latitude_str, ',');
        double longitude = std::stod(longitude_str);
        double latitude = std::stod(latitude_str);
        graph->addVertex(id_str);
        auto v = graph->findVertex(id_str);
        v->setLongitude(longitude);
        v->setLatitude(latitude);
    }

    // Parse edges
    std::ifstream edges_file(edges_filepath);
    if (!edges_file.is_open()) {
        std::cerr << "Error: Unable to open edges file " << edges_filepath << std::endl;
        return nullptr;
    }
    std::string edges_line;
    while(getline(edges_file, edges_line)) {
        std::istringstream edges_iss(edges_line);
        std::string source_id_str, dest_id_str, weight_str;
        getline(edges_iss, source_id_str, ',');
        getline(edges_iss, dest_id_str, ',');
        getline(edges_iss, weight_str, ',');
        double weight = std::stod(weight_str);
        graph->addEdge(source_id_str, dest_id_str, weight);
        graph->addEdge(dest_id_str, source_id_str, weight);
    }

    return graph;
}