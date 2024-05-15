#ifndef PROJETO_2_ACTIONS_H
#define PROJETO_2_ACTIONS_H

#include <vector>
#include <climits>
#include "graph.h"
#include <unordered_map>
#include <queue>

/* ===========================================4.1===============================================*/
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ hash2;
    }
};
using MemoizationTable = std::unordered_map<std::pair<Vertex*, int>, double, PairHash>;
double TSPBacktracking(Graph* graph);
double TSPHeldKarp(Graph* graph, Vertex* curr, int bitmask, MemoizationTable& memoization);
/* ===========================================4.2===============================================*/
double TSPApproximation(Graph* graph);
double findTriangularDistance(Vertex* v1, Vertex* v2, Graph* graph);
double TSPTriangularApproximation(Graph* graph);
/* ===========================================4.3===============================================*/

#endif //PROJETO_2_ACTIONS_H
