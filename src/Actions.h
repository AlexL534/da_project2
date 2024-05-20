/**
 * @file actions.h
 * @brief Header file for the Actions module.
 *
 * This module contains the functions that implement the algorithms to solve the Traveling Salesman Problem (TSP).
 * The algorithms implemented are:
 *  - Backtracking
 *  - Held-Karp
 *  - Triangular Approximation
 *  - Christofides
 *  - Nearest Neighbor
 *  - Hybrid MST and Nearest Neighbor
 *
 * The module also contains helper functions to generate Minimum Spanning Trees (MST), perfect matchings, and Eulerian circuits.
 */

#ifndef PROJETO_2_ACTIONS_H
#define PROJETO_2_ACTIONS_H

#include <vector>
#include <climits>
#include <algorithm>
#include <limits>
#include <random>
#include <cmath>
#include "graph.h"
#include <unordered_map>
#include <queue>

/* ===========================================4.1===============================================*/

/**
 * @brief Hash function for pairs used in memoization.
 */
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ hash2;
    }
};

using MemoizationTable = std::unordered_map<std::pair<Vertex*, int>, double, PairHash>;

/**
 * @brief Implements the backtracking algorithm to solve the TSP.
 * The time complexity of this algorithm is O(n^2 * 2^n).
 *
 * This function initiates the TSP solution using a backtracking approach starting and ending at node "0".
 *
 * @param graph Pointer to the graph representing the nodes and edges.
 * @return The minimum path cost calculated by the backtracking algorithm.
 */
double TSPBacktracking(Graph* graph);

/**
 * @brief Recursive helper function for TSPBacktracking.
 * The time complexity of this algorithm is O(n^2 * 2^n).
 *
 * Uses memoization to efficiently calculate the minimum path cost.
 *
 * @param graph Pointer to the graph.
 * @param curr Current vertex in the tour.
 * @param bitmask Bitmask representing visited nodes.
 * @param memoization Memoization table to store already computed costs.
 * @return The minimum path cost from the current vertex.
 */
double TSPHeldKarp(Graph* graph, Vertex* curr, int bitmask, MemoizationTable& memoization);

/* ===========================================4.2===============================================*/

/**
 * @brief Generates a Minimum Spanning Tree (MST) using Prim's algorithm.
 * The time complexity of this algorithm is O((V + E) log V).
 *
 * @param graph Pointer to the graph.
 * @param startVertexLabel Label of the starting vertex.
 * @return The MST as a graph.
 */
Graph primMST(Graph* graph, const std::string& startVertexLabel);

/**
 * @brief Performs a pre-order walk on the MST to generate a tour.
 * The time complexity of this algorithm is O(V + E).
 *
 * @param vertex Pointer to the current vertex.
 * @param visited Set of visited vertices.
 * @param preOrderList List to store the pre-order walk vertices.
 */
void preOrderWalk(Vertex* vertex, std::unordered_set<Vertex*>& visited, std::vector<Vertex*>& preOrderList);

/**
 * @brief Connects all edges in the graph based on the Haversine distance.
 * The time complexity of this algorithm is O(V^2).
 *
 * @param graph Pointer to the graph.
 */
void connectAllEdges(Graph *graph);

/**
 * @brief Solves the TSP using the Triangular Approximation heuristic.
 * The time complexity of this algorithm is O(V^2 log V).
 *
 * @param graph Pointer to the graph.
 * @return The approximate minimum path cost.
 */
double TSPTriangularApproximation(Graph* graph);

/* ===========================================Other heuristics===============================================*/

/**
 * @brief Finds a perfect matching on the odd degree vertices of the MST.
 * The time complexity of this algorithm is O(V^3).
 *
 * @param MST Pointer to the MST graph.
 * @return Graph representing the minimum weight perfect matching.
 */
Graph findPerfectMatching(Graph *MST);

/**
 * @brief Combines the MST and minimum weight perfect matching to form a multigraph.
 * The time complexity of this algorithm is O(V + E).
 *
 * @param MST Pointer to the MST graph.
 * @param MWPM Pointer to the minimum weight perfect matching graph.
 * @return Combined multigraph.
 */
Graph combineMSTAndPM(const Graph* MST, Graph *MWPM);

/**
 * @brief Finds an Eulerian circuit in the given multigraph.
 * The time complexity of this algorithm is O(V + E).
 *
 * @param multigraph Pointer to the multigraph.
 * @return Vector of vertices representing the Eulerian circuit.
 */
std::vector<Vertex*> findEulerianCircuit(Graph* multigraph);

/**
 * @brief Creates a Hamiltonian circuit by shortcutting the Eulerian circuit.
 * The time complexity of this algorithm is O(V).
 *
 * @param eulerianCircuit Vector of vertices representing the Eulerian circuit.
 * @return Vector of vertices representing the Hamiltonian circuit.
 */
std::vector<Vertex*> shortcutEulerianCircuit(const std::vector<Vertex*>& eulerianCircuit);

/**
 * @brief Calculates the total cost of a given Hamiltonian circuit.
 * The time complexity of this algorithm is O(V).
 *
 * @param hamiltonianCircuit Vector of vertices representing the Hamiltonian circuit.
 * @param graph Pointer to the graph.
 * @return The total cost of the Hamiltonian circuit.
 */
double calculateTotalCost(const std::vector<Vertex*>& hamiltonianCircuit, Graph* graph);

/**
 * @brief Solves the TSP using Christofides' algorithm.
 * The time complexity of this algorithm is O(V^3).
 *
 * @param graph Pointer to the graph.
 * @return The approximate minimum path cost.
 */
double TSPChristofides(Graph* graph);

/* ===========================================4.4===============================================*/

/**
 * @brief Solves the TSP using the Nearest Neighbor heuristic.
 * The time complexity of this algorithm is O(V^2).
 *
 * @param graph Pointer to the graph.
 * @param start Label of the starting vertex.
 * @param totalCost Reference to store the total cost of the tour.
 * @return Vector of vertices representing the tour.
 */
std::vector<Vertex*> nearestNeighborTSP(Graph* graph, const std::string& start, double& totalCost, int size, bool isFC);

/**
 * @brief Solves the TSP using a hybrid of MST and Nearest Neighbor heuristics.
 * The time complexity of this algorithm is O(V^2 log V).
 *
 * @param graph Pointer to the graph.
 * @param start Label of the starting vertex.
 * @param totalCost Reference to store the total cost of the tour.
 * @return The approximate minimum path cost.
 */
double hybridMSTAndNNTSP(Graph* graph, const std::string& start, double& totalCost);

double NNTSP(Graph* graph, const string& start, double& totalCost, vector<Vertex*>& solution, int size, bool isFC);

double SimulatedAnnealing(Graph* graph, vector<Vertex*> solution, double& totalCost, double initialTemp, double finalTemp, double alpha, int maxIter);

/* ===========================================Extended-Christofides===============================================*/

/**
 * @brief Runs the Floyd-Warshall algorithm to find shortest paths between all pairs of vertices.
 * The time complexity of this algorithm is O(V^3).
 *
 * @param graph Pointer to the graph.
 * @param shortestPaths A map to store the shortest paths.
 */
void floydWarshall(Graph* graph, std::unordered_map<Vertex*, std::unordered_map<Vertex*, double>>& shortestPaths);

/**
 * @brief Finds vertices with odd degree in the graph.
 * The time complexity of this algorithm is O(V).
 *
 * @param graph Pointer to the graph.
 * @return Vector of vertices with odd degree.
 */
std::vector<Vertex*> findOddDegreeVertices(Graph* graph);

/**
 * @brief Finds the minimum cost perfect matching among vertices with odd degree.
 * The time complexity of this algorithm is O(V^3).
 *
 * @param mst Pointer to the MST graph.
 * @return Vector of edges representing the minimum cost perfect matching.
 */
std::vector<Edge*> minimumCostPerfectMatching(Graph* mst);

/**
 * @brief Combines the MST and minimum weight perfect matching to form a multigraph.
 * The time complexity of this algorithm is O(V^2).
 *
 * @param MST Pointer to the MST graph.
 * @param oddDegreeVertices Vector of vertices with odd degree.
 * @param shortestPaths A map containing the shortest paths between vertices.
 * @return Combined multigraph.
 */
Graph combineMSTAndMWPM(const Graph* MST, const std::vector<Vertex*>& oddDegreeVertices, std::unordered_map<Vertex*, std::unordered_map<Vertex*, double>>& shortestPaths);

/**
 * @brief Finds an Eulerian walk in the given multigraph.
 * The time complexity of this algorithm is O(V + E).
 *
 * @param multigraph Pointer to the multigraph.
 * @return Vector of vertices representing the Eulerian walk.
 */
std::vector<Vertex*> findEulerianWalk(Graph* multigraph);

/**
 * @brief Substitutes the shortest path for each edge in the Eulerian circuit.
 * The time complexity of this algorithm is O(V^2).
 *
 * @param eulerianCircuit Vector of vertices representing the Eulerian circuit.
 * @param graph Pointer to the graph.
 * @param shortestPaths A map containing the shortest paths between vertices.
 * @return Vector of vertices representing the Hamiltonian circuit.
 */
std::vector<Vertex*> substituteShortestPath(const std::vector<Vertex*>& eulerianCircuit, Graph* graph, std::unordered_map<Vertex*, std::unordered_map<Vertex*, double>>& shortestPaths);

/**
 * @brief Finds the shortest path between two vertices in the graph.
 * The time complexity of this algorithm is O(V^2).
 *
 * @param start Pointer to the starting vertex.
 * @param end Pointer to the ending vertex.
 * @param graph Pointer to the graph.
 * @return Vector of vertices representing the shortest path.
 */
std::vector<Vertex*> findShortestPath(Vertex* start, Vertex* end, Graph* graph);

/**
 * @brief Solves the TSP using an extended version of Christofides' algorithm.
 * The time complexity of this algorithm is O(V^3).
 *
 * @param graph Pointer to the graph.
 * @param startVertexLabel Label of the starting vertex.
 * @return The approximate minimum path cost.
 */
double TSPExtendedChristofides(Graph* graph, const std::string& startVertexLabel);

using MatchedPairs = std::vector<std::pair<Vertex*, Vertex*>>;

/**
 * @brief Solves the minimum cost perfect matching problem using the Hungarian algorithm.
 * The time complexity of this algorithm is O(V^3).
 *
 * @param oddDegreeVertices Vector of vertices with odd degree.
 * @param shortestPaths A map containing the shortest paths between vertices.
 * @return Vector of pairs representing the minimum cost perfect matching.
 */
MatchedPairs hungarianAlgorithm(const std::vector<Vertex*>& oddDegreeVertices, std::unordered_map<Vertex*, std::unordered_map<Vertex*, double>>& shortestPaths);

#endif // PROJETO_2_ACTIONS_H
