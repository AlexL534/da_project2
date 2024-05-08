#include "Actions.h"

double TSPBacktracking(Graph* graph) {
    MemoizationTable memoization;
    Vertex* start = *graph->getVertexSet().begin();
    int bitmask = 1 << stoi(start->getInfo());

    double min_path_cost = TSPHeldKarp(graph, start, bitmask, memoization);
    return min_path_cost;
}

double TSPHeldKarp(Graph* graph, Vertex* curr, int bitmask, MemoizationTable& memoization) {
    // Base case: all vertices visited
    if (bitmask == (1 << graph->getNumVertex()) - 1) {
        Edge* returnEdge = graph->findEdge(curr->getInfo(), "0");
        if (returnEdge) {
            return returnEdge->getWeight();
        }
        return INT_MAX; // Return a large value if no edge back to starting vertex
    }

    // If the solution for this subproblem has already been computed, return it from the memoization table
    auto key = std::make_pair(curr, bitmask);
    if (memoization.find(key) != memoization.end()) {
        return memoization[key];
    }

    double min_path_cost = INT_MAX;

    for (Edge* edge : curr->getAdj()) {
        Vertex* next = edge->getDest();
        int nextIndex = stoi(next->getInfo());
        if (!(bitmask & (1 << nextIndex))) {
            double subproblemCost = edge->getWeight() + TSPHeldKarp(graph, next, bitmask | (1 << nextIndex), memoization);
            min_path_cost = std::min(min_path_cost, subproblemCost);
        }
    }

    memoization[key] = min_path_cost;
    return min_path_cost;
}