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
Graph primMST(Graph* graph, const string& startVertexLabel);
void preOrderWalk(Vertex* vertex, std::unordered_set<Vertex*>& visited, std::vector<Vertex*>& preOrderList);
void connectAllEdges(Graph *graph);
double TSPTriangularApproximation(Graph* graph);
/* ===========================================2-opt===============================================*/
std::vector<Vertex*> initializeTour(Graph* graph);
double calculateTourLength(Graph* graph, const std::vector<Vertex*>& tour);
std::vector<Vertex*> optSwap(const std::vector<Vertex*>& tour, size_t i, size_t j);
double TSP2Opt(Graph* graph);
double calculateDeltaTourLength(Graph* graph, const std::vector<Vertex*>& tour, size_t i, size_t j);
/* ===========================================Lin-Kernighan===============================================*/
//also uses calculateTourLength
std::vector<Vertex*> twoOptSwap(const std::vector<Vertex*>& tour, int i, int k);
std::vector<Vertex*> linKernighan(Graph* graph, std::vector<Vertex*> initialTour);
std::vector<Vertex*> initializeGreedyTour(Graph* graph);
double TSP_LinKernighan(Graph* graph);

#endif //PROJETO_2_ACTIONS_H