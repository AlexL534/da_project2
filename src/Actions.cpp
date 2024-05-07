#include "Actions.h"

/* ===========================================4.1===============================================*/
double TSPBacktracking(Graph* graph) {
    double min_path_cost = INT_MAX;

    for (Vertex* vertex : graph->getVertexSet()) {
        vertex->setVisited(false);
    }

    for (Vertex* start : graph->getVertexSet()) {
        start->setVisited(true);
        std::vector<Vertex*> path;
        path.push_back(start);
        TSPUtil(graph, start, path, 0, min_path_cost);
        start->setVisited(false);
    }
    return min_path_cost;
}

void TSPUtil(Graph* graph, Vertex* curr, std::vector<Vertex*>& path, double curr_cost, double& min_path_cost) {
    // Base case: If all nodes have been visited and there is an edge back to the starting node
    if (path.size() == graph->getVertexSet().size() && graph->findEdge(curr->getInfo(), path[0]->getInfo())) {
        double total_cost = curr_cost + graph->findEdge(curr->getInfo(), path[0]->getInfo())->getWeight();
        min_path_cost = std::min(min_path_cost, total_cost);
        return;
    }

    if (curr_cost >= min_path_cost) {
        return;
    }

    for (Edge* edge : curr->getAdj()) {
        Vertex* next = edge->getDest();
        if (!next->isVisited()) {
            next->setVisited(true);
            path.push_back(next);
            TSPUtil(graph, next, path, curr_cost + edge->getWeight(), min_path_cost);
            path.pop_back();  // Backtrack
            next->setVisited(false);
        }
    }
}