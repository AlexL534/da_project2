#include "Actions.h"
#include "Utils.h"

/* ===========================================4.1===============================================*/
double TSPBacktracking(Graph* graph) {
    MemoizationTable memoization;
    unordered_map<string, Vertex *> vertexMap = graph->getVertexMap();
    Vertex* start = vertexMap["0"];
    int bitmask = 1 << stoi(start->getInfo());

    double min_path_cost = TSPHeldKarp(graph, start, bitmask, memoization);
    return min_path_cost;
}

double TSPHeldKarp(Graph* graph, Vertex* curr, int bitmask, MemoizationTable& memoization) {
    if (bitmask == (1 << graph->getNumVertex()) - 1) {
        Edge* returnEdge = graph->findEdge(curr->getInfo(), "0");
        if (returnEdge) {
            return returnEdge->getWeight();
        }
        return INT_MAX;
    }

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
/* ===========================================4.2===============================================*/
Graph primMST(Graph* graph, const string& startVertexLabel) {

    priority_queue<Edge*, vector<Edge*>, CompareWeight> pq;

    unordered_map<string, bool> visited;

    auto vertexMap = graph->getVertexMap();
    for (const auto& pair : vertexMap) {
        visited[pair.first] = false;
    }

    Vertex* startVertex = graph->findVertex(startVertexLabel);
    visited[startVertexLabel] = true;

    for (Edge* edge : startVertex->getAdj()) {
        pq.push(edge);
    }

    Graph MST;

    while (!pq.empty()) {
        Edge* minEdge = pq.top();
        pq.pop();

        Vertex* src = minEdge->getSource();
        Vertex* dest = minEdge->getDest();

        if (visited[dest->getInfo()]) {
            continue;
        }

        MST.addVertex(src->getInfo());
        MST.addVertex(dest->getInfo());
        MST.addEdge(src->getInfo(), dest->getInfo(), minEdge->getWeight());

        visited[dest->getInfo()] = true;

        for (Edge* edge : dest->getAdj()) {
            if (!visited[edge->getDest()->getInfo()]) {
                pq.push(edge);
            }
        }
    }

    return MST;
}

void preOrderWalk(Vertex* vertex, std::unordered_set<Vertex*>& visited, std::vector<Vertex*>& preOrderList) {
    if (vertex == nullptr) return;

    preOrderList.push_back(vertex);
    visited.insert(vertex);

    for (Edge* edge : vertex->getAdj()) {
        Vertex* nextVertex = edge->getDest();
        if (visited.find(nextVertex) == visited.end()) {
            preOrderWalk(nextVertex, visited, preOrderList);
        }
    }
}

void connectAllEdges(Graph *graph) {
    auto vertexMap = graph->getVertexMap();

    for (const auto& pair : vertexMap) {
        auto vertex = pair.second;
        const std::vector<Edge*>& adjEdges = vertex->getAdj();
        for (Edge* edge : adjEdges) {
            Vertex* adjacentVertex = edge->getDest();
            if (!graph->findEdge(vertex->getInfo(), adjacentVertex->getInfo())) {
                double dist = haversineDistance(vertex->getLatitude(), vertex->getLongitude(),adjacentVertex->getLatitude(), adjacentVertex->getLongitude());
                graph->addEdge(vertex->getInfo(), adjacentVertex->getInfo(), dist);
            }
        }
    }
}

double TSPTriangularApproximation(Graph* graph) {
    double minPath = 0;

    connectAllEdges(graph);

    Graph MST = primMST(graph, "0");
    std::unordered_set<Vertex*> visited;
    std::vector<Vertex*> preOrderList;
    preOrderWalk(MST.findVertex("0"), visited, preOrderList);
    std::unordered_set<Vertex*> visitedInTour;
    std::vector<Vertex*> tour;
    for (Vertex* vertex : preOrderList) {
        if (visitedInTour.find(vertex) == visitedInTour.end()) {
            tour.push_back(vertex);
            visitedInTour.insert(vertex);
        }
    }

    minPath += haversineDistance(tour.back()->getLatitude(), tour.back()->getLongitude(),
                                 tour.front()->getLatitude(), tour.front()->getLongitude());
    tour.push_back(tour.front());

    for (size_t i = 0; i < tour.size() - 1; ++i) {
        Edge* edge = graph->findEdge(tour[i]->getInfo(), tour[i + 1]->getInfo());
        if (edge) {
            minPath += edge->getWeight();
        }
    }

    return minPath;
}
/* ===========================================2-opt===============================================*/
std::vector<Vertex*> initializeTour(Graph* graph) {
    std::vector<Vertex*> tour;

    // Start from vertex "0"
    Vertex* currentVertex = graph->findVertex("0");
    currentVertex->setVisited(true);
    tour.push_back(currentVertex);

    // Greedily select the next vertex based on the closest unvisited neighbor
    while (tour.size() < graph->getNumVertex()) {
        Edge* minEdge = nullptr;
        double minDistance = std::numeric_limits<double>::max();

        for (Edge* edge : currentVertex->getAdj()) {
            Vertex* neighbor = edge->getDest();
            if (!neighbor->isVisited()) {
                if (edge->getWeight() < minDistance) {
                    minEdge = edge;
                    minDistance = edge->getWeight();
                }
            }
        }

        if (minEdge) {
            Vertex* nextVertex = minEdge->getDest();
            nextVertex->setVisited(true);
            tour.push_back(nextVertex);
            currentVertex = nextVertex;
        } else {
            // If there are no unvisited neighbors, break out of the loop
            break;
        }
    }

    // Add the starting vertex to the end to complete the tour
    tour.push_back(tour.front());

    return tour;
}

double calculateTourCost(Graph* graph, const std::vector<Vertex*>& tour) {
    double cost = 0.0;
    for (size_t i = 0; i < tour.size() - 1; ++i) {
        Edge* edge = graph->findEdge(tour[i]->getInfo(), tour[i + 1]->getInfo());
        if (edge) {
            cost += edge->getWeight();
        }
    }
    // Add distance from the last vertex back to the starting vertex
    Edge* returnEdge = graph->findEdge(tour.back()->getInfo(), tour.front()->getInfo());
    if (returnEdge) {
        cost += returnEdge->getWeight();
    }
    return cost;
}

std::vector<Vertex*> optSwap(const std::vector<Vertex*>& tour, size_t i, size_t j) {
    std::vector<Vertex*> newTour = tour;
    while (i < j) {
        std::swap(newTour[i], newTour[j]);
        ++i;
        --j;
    }
    return newTour;
}

double TSP2Opt(Graph* graph) {
    // Initialize the initial tour, e.g., using a greedy algorithm
    std::vector<Vertex*> tour = initializeTour(graph);

    // Calculate the initial tour cost
    double initialTourCost = calculateTourCost(graph, tour);

    // Perform 2-opt optimization until no improvement is possible
    bool improvement = true;
    while (improvement) {
        improvement = false;
        double bestDiffTourCost = 0.0;
        size_t bestI, bestJ;

        for (size_t i = 0; i < tour.size() - 1; ++i) {
            for (size_t j = i + 1; j < tour.size(); ++j) {
                // Apply 2-opt swap
                std::vector<Vertex*> newTour = optSwap(tour, i, j);

                // Calculate the difference in tour cost
                double diffTourCost = calculateTourCost(graph, newTour) - initialTourCost;

                // If the new tour is shorter, update the tour
                if (diffTourCost < bestDiffTourCost) {
                    bestDiffTourCost = diffTourCost;
                    bestI = i;
                    bestJ = j;
                    improvement = true;
                }
            }
        }

        // If an improvement is found, apply the best 2-opt swap
        if (improvement) {
            tour = optSwap(tour, bestI, bestJ);
            initialTourCost += bestDiffTourCost;
        }
    }

    // Return the final tour cost
    return initialTourCost;
}

/* ===========================================Lin-Kernighan===============================================*/
std::vector<Vertex*> twoOptSwap(const std::vector<Vertex*>& tour, int i, int k) {
    std::vector<Vertex*> newTour = tour;
    std::reverse(newTour.begin() + i, newTour.begin() + k + 1);
    return newTour;
}

std::vector<Vertex*> linKernighan(Graph* graph, std::vector<Vertex*> initialTour) {
    bool improvement = true;
    std::vector<Vertex*> bestTour = initialTour;
    double bestCost = calculateTourCost(graph, bestTour);

    while (improvement) {
        improvement = false;
        for (size_t i = 1; i < bestTour.size() - 1; ++i) {
            for (size_t k = i + 1; k < bestTour.size(); ++k) {
                std::vector<Vertex*> newTour = twoOptSwap(bestTour, i, k);
                double newCost = calculateTourCost(graph, newTour);
                if (newCost < bestCost) {
                    bestTour = newTour;
                    bestCost = newCost;
                    improvement = true;
                }
            }
        }
    }

    return bestTour;
}

std::vector<Vertex*> initializeGreedyTour(Graph* graph) {
    std::vector<Vertex*> tour;
    std::unordered_set<Vertex*> visited;

    Vertex* startVertex = graph->findVertex("0");
    tour.push_back(startVertex);
    visited.insert(startVertex);

    Vertex* currentVertex = startVertex;
    while (tour.size() < graph->getNumVertex()) {
        Edge* minEdge = nullptr;
        double minDistance = std::numeric_limits<double>::max();

        for (Edge* edge : currentVertex->getAdj()) {
            Vertex* neighbor = edge->getDest();
            if (visited.find(neighbor) == visited.end() && edge->getWeight() < minDistance) {
                minEdge = edge;
                minDistance = edge->getWeight();
            }
        }

        if (minEdge) {
            Vertex* nextVertex = minEdge->getDest();
            tour.push_back(nextVertex);
            visited.insert(nextVertex);
            currentVertex = nextVertex;
        } else {
            break;
        }
    }

    tour.push_back(startVertex); // Complete the tour
    return tour;
}

double TSP_LinKernighan(Graph* graph) {
    std::vector<Vertex*> initialTour = initializeGreedyTour(graph);
    std::vector<Vertex*> optimizedTour = linKernighan(graph, initialTour);
    double tourCost = calculateTourCost(graph, optimizedTour);
    return tourCost;
}