#include "Actions.h"
#include "utils.h"

/* ===========================================4.1===============================================*/
double TSPHeldKarp(Graph* graph, Vertex* curr, int bitmask, MemoizationTable& memoization, const std::unordered_map<std::string, int>& vertexIndex) {
    // Base case: all vertices visited
    if (bitmask == (1 << graph->getNumVertex()) - 1) {
        Edge* returnEdge = graph->findEdge(curr->getInfo(), "0");
        if (returnEdge) {
            return returnEdge->getWeight();
        }
        return std::numeric_limits<double>::max(); // Return a large value if no edge back to starting vertex
    }

    // If the solution for this subproblem has already been computed, return it from the memoization table
    auto key = std::make_pair(curr, bitmask);
    if (memoization.find(key) != memoization.end()) {
        return memoization[key];
    }

    double min_path_cost = std::numeric_limits<double>::max();

    for (Edge* edge : curr->getAdj()) {
        Vertex* next = edge->getDest();
        int nextIndex = vertexIndex.at(next->getInfo());
        if (!(bitmask & (1 << nextIndex))) {
            double subproblemCost = edge->getWeight() + TSPHeldKarp(graph, next, bitmask | (1 << nextIndex), memoization, vertexIndex);
            min_path_cost = std::min(min_path_cost, subproblemCost);
        }
    }

    memoization[key] = min_path_cost;
    return min_path_cost;
}

double TSPBacktracking(Graph* graph) {
    MemoizationTable memoization;
    std::unordered_map<std::string, int> vertexIndex;
    int index = 0;
    for (const auto& vertex : graph->getVertexSet()) {
        vertexIndex[vertex->getInfo()] = index++;
    }

    Vertex* start = *graph->getVertexSet().begin();
    int startBitmask = 1 << vertexIndex[start->getInfo()];

    double min_path_cost = TSPHeldKarp(graph, start, startBitmask, memoization, vertexIndex);
    return min_path_cost;
}



/* ===========================================4.2===============================================*/
double findTriangularDistance(Vertex* v1, Vertex* v2, Graph* graph, std::unordered_map<Vertex*, std::vector<std::pair<Vertex*, double>>>& nearestNeighbors) {
    double direct_distance = std::numeric_limits<double>::max();

    // Check if the direct distance is given
    Edge* edge = graph->findEdge(v1->getInfo(), v2->getInfo());
    if (edge) {
        direct_distance = edge->getWeight();
    } else {
        // Calculate haversine distance if edge not found
        std::vector<double> coord1 = {v1->getLatitude(), v1->getLongitude()};
        std::vector<double> coord2 = {v2->getLatitude(), v2->getLongitude()};
        direct_distance = haversineDistance(coord1, coord2);
    }

    // Find the two nearest neighbors of v1 and v2 using precomputed data
    double nearest1 = std::numeric_limits<double>::max();
    double nearest2 = std::numeric_limits<double>::max();

    for (auto& neighbor : nearestNeighbors[v1]) {
        if (neighbor.first != v2) {
            nearest1 = std::min(nearest1, neighbor.second);
        }
    }

    for (auto& neighbor : nearestNeighbors[v2]) {
        if (neighbor.first != v1) {
            nearest2 = std::min(nearest2, neighbor.second);
        }
    }

    // Return the minimum of direct distance and sum of two nearest neighbors' distances
    return std::min(direct_distance, nearest1 + nearest2);
}

double TSPTriangularApproximation(Graph* graph) {
    double min_path_cost = 0.0;

    Vertex* start = graph->findVertex("0");
    std::vector<Vertex*> visited;
    visited.push_back(start);

    // Mark the start vertex as visited
    start->setVisited(true);

    // Precompute nearest neighbors for all vertices
    std::unordered_map<Vertex*, std::vector<std::pair<Vertex*, double>>> nearestNeighbors;
    for (Vertex* v : graph->getVertexSet()) {
        std::vector<std::pair<Vertex*, double>> neighbors;
        for (Edge* edge : v->getAdj()) {
            neighbors.push_back({edge->getDest(), edge->getWeight()});
        }
        nearestNeighbors[v] = neighbors;
    }

    // Iterate until all vertices are visited
    while (visited.size() < graph->getNumVertex()) {
        double min_distance = std::numeric_limits<double>::max();
        Vertex* nearest_neighbor = nullptr;

        // Iterate over unvisited vertices to find the nearest neighbor based on triangular inequality
        for (Vertex* v : graph->getVertexSet()) {
            if (!v->isVisited()) {
                double distance = findTriangularDistance(visited.back(), v, graph, nearestNeighbors);
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_neighbor = v;
                }
            }
        }

        // Add the nearest unvisited neighbor to the visited list and update the cost
        visited.push_back(nearest_neighbor);
        nearest_neighbor->setVisited(true);
        min_path_cost += min_distance;
    }

    // Add the distance from the last vertex to the start vertex to complete the cycle
    Edge* returnEdge = graph->findEdge(visited.back()->getInfo(), start->getInfo());
    if (returnEdge) {
        min_path_cost += returnEdge->getWeight();
    } else {
        // Calculate haversine distance if edge not found
        std::vector<double> last_coord = {visited.back()->getLatitude(), visited.back()->getLongitude()};
        std::vector<double> start_coord = {start->getLatitude(), start->getLongitude()};
        min_path_cost += haversineDistance(last_coord, start_coord);
    }

    return min_path_cost;
}

/* ===========================================4.3===============================================*/
