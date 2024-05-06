#ifndef PROJETO_2_PARSE_H
#define PROJETO_2_PARSE_H

#include <vector>
#include <map>
#include "Graph.h"

Graph* parseToyGraph(const std::string& filename);
Graph* parseExtraFullyConnectedGraph(const std::string& edges_filepath, const std::string& nodes_filepath);
Graph* parseRealWorldGraph(const std::string& edges_filepath, const std::string& nodes_filepath);

#endif //PROJETO_2_PARSE_H
