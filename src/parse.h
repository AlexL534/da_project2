#ifndef PROJETO_2_PARSE_H
#define PROJETO_2_PARSE_H

#include <vector>
#include <map>
#include "graph.h"

 /**
 * @brief Loads one of the graphs in "Toy-Graphs" directory from a file
 * @param filepath Path to file
 * @return Pointer to graph or nullptr if file not found
 */
Graph* parseToyGraph(const std::string& filepath);

 /**
 * @brief Loads one of the graphs in "Extra_Fully_Connected_Graphs" directory from a file
 * @param edges_filepath Path to file containing edges
 * @param nodes_filepath Path to file containing nodes
 * @return Pointer to graph or nullptr if file not found
 */
Graph* parseExtraFullyConnectedGraph(const std::string& edges_filepath, const std::string& nodes_filepath);

 /**
 * @brief Loads one of the graphs in "Real-world Graphs" directory from a file
 * @param edges_filepath Path to file containing edges
 * @param nodes_filepath Path to file containing nodes
 * @return Pointer to graph or nullptr if file not found
 */
Graph* parseRealWorldGraph(const std::string& edges_filepath, const std::string& nodes_filepath);

#endif //PROJETO_2_PARSE_H
