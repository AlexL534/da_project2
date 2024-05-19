/**
 * @file Actions.h
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
 * The time complexity of this algorithm is (O((V + E) log V))
 *
 * @param graph Pointer to the graph.
 * @param startVertexLabel Label of the starting vertex.
 * @return The MST as a graph.
 */
Graph primMST(Graph* graph, const std::string& startVertexLabel);

/**
 * @brief Performs a pre-order walk on the MST to generate a tour.
 * The time complexity of this algorithm is (O(V + E))
 *
 * @param vertex Pointer to the current vertex.
 * @param visited Set of visited vertices.
 * @param preOrderList List to store the pre-order walk vertices.
 */
void preOrderWalk(Vertex* vertex, std::unordered_set<Vertex*>& visited, std::vector<Vertex*>& preOrderList);

/**
 * @brief Connects all edges in the graph based on the Haversine distance.
 * The time complexity of this algorithm is (O(V^2))
 *
 * @param graph Pointer to the graph.
 */
void connectAllEdges(Graph *graph);

/**
 * @brief Solves the TSP using the Triangular Approximation heuristic.
 * The time complexity of this algorithm is (O(V^2 log V))
 *
 * @param graph Pointer to the graph.
 * @return The approximate minimum path cost.
 */
double TSPTriangularApproximation(Graph* graph);

/* ===========================================Other heuristics===============================================*/

/**
 * @brief Finds a perfect matching on the odd degree vertices of the MST.
 * The time complexity of this algorithm is (O(V^3))
 *
 * @param MST Pointer to the MST graph.
 * @return Graph representing the minimum weight perfect matching.
 */
Graph findPerfectMatching(Graph *MST);

/**
 * @brief Combines the MST and minimum weight perfect matching to form a multigraph.
 * The time complexity of this algorithm is (O(V + E))
 *
 * @param MST Pointer to the MST graph.
 * @param MWPM Pointer to the minimum weight perfect matching graph.
 * @return Combined multigraph.
 */
Graph combineMSTAndPM(const Graph* MST, Graph *MWPM);

/**
 * @brief Finds an Eulerian circuit in the given multigraph.
 * The time complexity of this algorithm is (O(V + E))
 *
 * @param multigraph Pointer to the multigraph.
 * @return Vector of vertices representing the Eulerian circuit.
 */
std::vector<Vertex*> findEulerianCircuit(Graph* multigraph);

/**
 * @brief Creates a Hamiltonian circuit by shortcutting the Eulerian circuit.
 * The time complexity of this algorithm is (O(V))
 *
 * @param eulerianCircuit Vector of vertices representing the Eulerian circuit.
 * @return Vector of vertices representing the Hamiltonian circuit.
 */
std::vector<Vertex*> shortcutEulerianCircuit(const std::vector<Vertex*>& eulerianCircuit);

/**
 * @brief Calculates the total cost of a given Hamiltonian circuit.
 * The time complexity of this algorithm is (O(V))
 *
 * @param hamiltonianCircuit Vector of vertices representing the Hamiltonian circuit.
 * @param graph Pointer to the graph.
 * @return The total cost of the Hamiltonian circuit.
 */
double calculateTotalCost(const std::vector<Vertex*>& hamiltonianCircuit, Graph* graph);

/**
 * @brief Solves the TSP using Christofides' algorithm.
 * The time complexity of this algorithm is (O(V^3))
 *
 * @param graph Pointer to the graph.
 * @return The approximate minimum path cost.
 */
double TSPChristofides(Graph* graph);

/* ===========================================4.4===============================================*/

/**
 * @brief Solves the TSP using the Nearest Neighbor heuristic.
 * The time complexity of this algorithm is (O(V^2))
 *
 * @param graph Pointer to the graph.
 * @param start Label of the starting vertex.
 * @param totalCost Reference to store the total cost of the tour.
 * @return Vector of vertices representing the tour.
 */
std::vector<Vertex*> nearestNeighborTSP(Graph* graph, const std::string& start, double& totalCost);

/**
 * @brief Solves the TSP using a hybrid of MST and Nearest Neighbor heuristics.
 * The time complexity of this algorithm is (O(V^2 log V))
 *
 * @param graph Pointer to the graph.
 * @param start Label of the starting vertex.
 * @param totalCost Reference to store the total cost of the tour.
 * @return The approximate minimum path cost.
 */
double hybridMSTAndNNTSP(Graph* graph, const std::string& start, double& totalCost);

#endif // PROJETO_2_ACTIONS_H
