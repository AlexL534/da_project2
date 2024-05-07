#ifndef PROJETO_2_ACTIONS_H
#define PROJETO_2_ACTIONS_H

#include <vector>
#include <climits>
#include "graph.h"

/* ===========================================4.1===============================================*/
double TSPBacktracking(Graph* graph);
void TSPUtil(Graph* graph, Vertex* curr, std::vector<Vertex*>& path, double curr_cost, double& min_path_cost);

#endif //PROJETO_2_ACTIONS_H
