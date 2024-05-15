#ifndef PROJETO_2_ACTIONS_H
#define PROJETO_2_ACTIONS_H

#include <vector>
#include <climits>
#include <algorithm>
#include <limits>
#include <xmath.h>
#include <cmath>
#include "graph.h"
#include <unordered_map>

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
void preOrderWalk(Vertex* vertex, std::unordered_set<Vertex*>& visited, std::vector<Vertex*>& preOrderList);
double TSPBacktracking(Graph* graph);
double TSPHeldKarp(Graph* graph, Vertex* curr, int bitmask, MemoizationTable& memoization);
map<vector<Vertex*>, double> TSPTriangularApproximation(Graph* graph);
set<Edge*> Prim(Graph* g);

#endif //PROJETO_2_ACTIONS_H
