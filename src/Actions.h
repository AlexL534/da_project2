#ifndef PROJETO_2_ACTIONS_H
#define PROJETO_2_ACTIONS_H

#include <vector>
#include <climits>
#include <algorithm>
#include <limits>
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
/* ===========================================Other heuristics===============================================*/
Graph findPerfectMatching(Graph *MST);
Graph combineMSTAndPM(const Graph* MST, Graph *MWPM);
std::vector<Vertex*> findEulerianCircuit(Graph* multigraph);
std::vector<Vertex*> shortcutEulerianCircuit(const std::vector<Vertex*>& eulerianCircuit);
double calculateTotalCost(const std::vector<Vertex*>& hamiltonianCircuit, Graph* graph);
double TSPChristofides(Graph* graph);
/* ===========================================4.4===============================================*/
vector<Vertex*> nearestNeighborTSP(Graph* graph, const string& start, double& totalCost);
double hybridMSTAndNNTSP(Graph* graph, const string& start, double& totalCost);
/* ===========================================Extended-Christofides===============================================*/
void floydWarshall(Graph* graph, unordered_map<Vertex*, unordered_map<Vertex*, double>>& shortestPaths);
vector<Vertex*> findOddDegreeVertices(Graph* graph);
vector<Edge*> minimumCostPerfectMatching(Graph* mst);
Graph combineMSTAndMWPM(const Graph* MST, const vector<Vertex*>& oddDegreeVertices, unordered_map<Vertex*, unordered_map<Vertex*, double>>& shortestPaths);
std::vector<Vertex*> findEulerianWalk(Graph* multigraph);
std::vector<Vertex*> substituteShortestPath(const std::vector<Vertex*>& eulerianCircuit, Graph* graph);
std::vector<Vertex*> findShortestPath(Vertex* start, Vertex* end, Graph* graph);
double TSPExtendedChristofides(Graph* graph, const string& startVertexLabel);
using MatchedPairs = vector<pair<Vertex*, Vertex*>>;
MatchedPairs hungarianAlgorithm(const vector<Vertex*>& oddDegreeVertices, unordered_map<Vertex*, unordered_map<Vertex*, double>>& shortestPaths);


#endif //PROJETO_2_ACTIONS_H