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
/* ===========================================4.3===============================================*/
Graph findPerfectMatching(Graph* MST) {
    Graph PM;

    // Check if MST is not nullptr
    if (!MST) {
        // Handle the case where MST is nullptr
        // For example, throw an exception or return an empty PM
        return PM;
    }

    // Find nodes with odd in-degrees in MST to form subgraph O
    std::list<Vertex*> oddVertices;
    for (const auto& vertexPair : MST->getVertexMap()) {
        Vertex* vertex = vertexPair.second;
        unsigned degree = vertex->getAdj().size();
        if (degree % 2 != 0) {
            oddVertices.push_back(vertex);
        }
    }

    // Greedily find perfect matching within subgraph O
    while (!oddVertices.empty()) {
        // Take the first odd vertex and find its nearest neighbor
        Vertex* first = oddVertices.front();
        double closestDist = std::numeric_limits<double>::max();
        Vertex* closestVertex = nullptr;
        for (auto it = std::next(oddVertices.begin()); it != oddVertices.end(); ++it) {
            Vertex* vertex = *it;
            Edge* edge = MST->findEdge(first->getInfo(), vertex->getInfo());
            double distance;
            if (edge) {
                distance = edge->getWeight();
            } else {
                distance = haversineDistance(first->getLatitude(), first->getLongitude(),
                                             vertex->getLatitude(), vertex->getLongitude());
            }
            if (distance < closestDist) {
                closestDist = distance;
                closestVertex = vertex;
            }
        }

        // Check if closestVertex is nullptr (no suitable match found)
        if (closestVertex) {
            PM.addEdge(first->getInfo(), closestVertex->getInfo(), closestDist);

            // Remove both vertices from the oddVertices list
            oddVertices.pop_front();
            oddVertices.remove(closestVertex);
        } else {
            oddVertices.pop_front(); // Remove first if no match found
        }
    }

    return PM;
}

Graph combineMSTAndPM(const Graph* MST, Graph *PM) {
    Graph multigraph = *MST;

    // Add edges from PM to the multigraph
    for (const auto& vertexPair : PM->getVertexMap()) {
        for (Edge* edge : vertexPair.second->getAdj()) {
            multigraph.addEdge(edge->getSource()->getInfo(), edge->getDest()->getInfo(), edge->getWeight());
        }
    }

    return multigraph;
}

std::vector<Vertex*> findEulerianCircuit(Graph* multigraph) {

    // Find Eulerian circuit using Hierholzer's algorithm
    std::vector<Vertex*> circuit;
    std::stack<Vertex*> stack;
    Vertex* currVertex = multigraph->findVertex("0");
    stack.push(currVertex);

    while (!stack.empty()) {
        Vertex* u = stack.top();
        if (!u->getAdj().empty()) {
            Vertex* v = u->getAdj().front()->getDest();
            stack.push(v);
            multigraph->removeEdge(u->getInfo(), v->getInfo()); // Remove edge from multigraph
        } else {
            circuit.push_back(u);
            stack.pop();
        }
    }

    return circuit;
}

std::vector<Vertex*> shortcutEulerianCircuit(const std::vector<Vertex*>& eulerianCircuit) {
    std::unordered_set<Vertex*> visited;
    std::vector<Vertex*> hamiltonianCircuit;

    for (Vertex* vertex : eulerianCircuit) {
        if (visited.find(vertex) == visited.end()) {
            hamiltonianCircuit.push_back(vertex);
            visited.insert(vertex);
        }
    }

    return hamiltonianCircuit;
}

double calculateTotalCost(const std::vector<Vertex*>& hamiltonianCircuit, Graph* graph) {
    double totalCost = 0;
    // Iterate over the vertices in the Hamiltonian circuit
    for (size_t i = 0; i < hamiltonianCircuit.size(); ++i) {
        // Get the current and next vertices
        Vertex* currentVertex = hamiltonianCircuit[i];
        Vertex* nextVertex = hamiltonianCircuit[(i + 1) % hamiltonianCircuit.size()];  // Wrap around for the last vertex

        // Find the edge between currentVertex and nextVertex in the graph
        Edge* edge = graph->findEdge(currentVertex->getInfo(), nextVertex->getInfo());
        if (edge) {
            // If edge weight is specified, use it
            totalCost += edge->getWeight();
        } else {
            // If edge weight is not specified, calculate the Haversine distance
            double distance = haversineDistance(currentVertex->getLatitude(), currentVertex->getLongitude(),
                                                nextVertex->getLatitude(), nextVertex->getLongitude());
            totalCost += distance;
        }
    }

    return totalCost;
}


double TSPChristofides(Graph* graph) {
    // Step 1: Find Minimum Spanning Tree (MST)
    Graph MST = primMST(graph, "0");

    // Step 2: Find Minimum Weight Perfect Matching (PM)
    Graph PM = findPerfectMatching(&MST);

    // Step 3: Combine MST and MWPM
    Graph multigraph = combineMSTAndPM(&MST, &PM);

    // Step 4: Find Eulerian Circuit
    std::vector<Vertex*> eulerianCircuit = findEulerianCircuit(&multigraph);

    // Step 5: Shortcut Eulerian Circuit to Hamiltonian Circuit
    std::vector<Vertex*> hamiltonianCircuit = shortcutEulerianCircuit(eulerianCircuit);

    // Step 6: Calculate Total Cost of Hamiltonian Circuit
    double totalCost = calculateTotalCost(hamiltonianCircuit, graph);

    return totalCost;
}

/* ===========================================4.4===============================================*/
vector<Vertex*> nearestNeighborTSP(Graph* graph, const string& start, double& totalCost) {
    unordered_set<Vertex*> visited;
    vector<Vertex*> path;
    Vertex* current = graph->findVertex(start);
    visited.insert(current);
    path.push_back(current);
    totalCost = 0;

    while (visited.size() < graph->getNumVertex()) {
        double minCost = numeric_limits<double>::max();
        Vertex* nextNode = nullptr;

        for (Edge* edge : current->getAdj()) {
            Vertex* neighbor = edge->getDest();
            if (visited.find(neighbor) == visited.end() && edge->getWeight() < minCost) {
                minCost = edge->getWeight();
                nextNode = neighbor;
            }
        }

        if (nextNode == nullptr) return {}; // No feasible TSP path exists

        visited.insert(nextNode);
        path.push_back(nextNode);
        totalCost += minCost;
        current = nextNode;

    }

    // Add the cost of returning to the start vertex
    for (Edge* edge : current->getAdj()) {
        if (edge->getDest()->getInfo() == start) {
            totalCost += edge->getWeight();
            path.push_back(graph->findVertex(start));
            break;
        }
    }

    return path;
}

double hybridMSTAndNNTSP(Graph* graph, const string& start, double& totalCost) {
    vector<Vertex*> nnPath = nearestNeighborTSP(graph, start, totalCost);
    return totalCost;
}

/* ===========================================Extended Christofides===============================================*/
void floydWarshall(Graph* graph, unordered_map<Vertex*, unordered_map<Vertex*, double>>& shortestPaths) {
    unordered_map<string, Vertex*> vertexMap = graph->getVertexMap();

    // Initialize distances with infinity
    for (auto& pair1 : vertexMap) {
        Vertex* v1 = pair1.second;
        shortestPaths[v1] = unordered_map<Vertex*, double>();
        for (auto& pair2 : vertexMap) {
            Vertex* v2 = pair2.second;
            shortestPaths[v1][v2] = numeric_limits<double>::infinity();
        }
        shortestPaths[v1][v1] = 0; // Distance from a vertex to itself is 0
    }

    // Update distances based on direct edges
    for (auto& pair : vertexMap) {
        Vertex* v1 = pair.second;
        for (Edge* edge : v1->getAdj()) {
            Vertex* v2 = edge->getDest();
            shortestPaths[v1][v2] = edge->getWeight();
        }
    }

    // Floyd-Warshall algorithm
    for (auto& k_pair : vertexMap) {
        Vertex* k = k_pair.second;
        for (auto& i_pair : vertexMap) {
            Vertex* i = i_pair.second;
            for (auto& j_pair : vertexMap) {
                Vertex* j = j_pair.second;
                if (shortestPaths[i][k] + shortestPaths[k][j] < shortestPaths[i][j]) {
                    shortestPaths[i][j] = shortestPaths[i][k] + shortestPaths[k][j];
                }
            }
        }
    }
}

vector<Vertex*> findOddDegreeVertices(Graph* graph) {
    unordered_map<string, Vertex*> vertexMap = graph->getVertexMap();
    vector<Vertex*> oddDegreeVertices;

    // Count degrees of vertices
    unordered_map<Vertex*, int> degreeCount;
    for (const auto& pair : vertexMap) {
        Vertex* vertex = pair.second;
        degreeCount[vertex] = vertex->getAdj().size(); // Assuming getAdj() returns list of adjacent vertices
    }

    // Find vertices with odd degrees
    for (const auto& pair : degreeCount) {
        Vertex* vertex = pair.first;
        int degree = pair.second;
        if (degree % 2 != 0) {
            oddDegreeVertices.push_back(vertex);
        }
    }

    return oddDegreeVertices;
}

vector<Edge*> minimumCostPerfectMatching(Graph* mst) {
    vector<Edge*> matchingEdges;

    // Priority queue to store edges sorted by weight
    priority_queue<Edge*, vector<Edge*>, CompareWeight> pq;

    // Get odd degree vertices from MST
    unordered_map<string, Vertex*> vertexMap = mst->getVertexMap();
    vector<Vertex*> oddDegreeVertices;
    unordered_map<Vertex*, int> degreeCount;
    for (const auto& pair : vertexMap) {
        Vertex* vertex = pair.second;
        int degree = vertex->getAdj().size();
        degreeCount[vertex] = degree;
        if (degree % 2 != 0) {
            oddDegreeVertices.push_back(vertex);
        }
    }

    // Process odd degree vertices to find matching edges
    for (size_t i = 0; i < oddDegreeVertices.size(); ++i) {
        Vertex* u = oddDegreeVertices[i];
        for (size_t j = i + 1; j < oddDegreeVertices.size(); ++j) {
            Vertex* v = oddDegreeVertices[j];
            Edge* edge = mst->findEdge(u->getInfo(), v->getInfo()); // Assuming findEdge returns the edge between u and v
            if (edge) {
                pq.push(edge);
            }
        }
    }

    // Perform matching by adding edges from priority queue to matching set
    while (!pq.empty()) {
        Edge* edge = pq.top();
        pq.pop();
        matchingEdges.push_back(edge); // Add edge to matching
    }

    return matchingEdges;
}

// Define a type alias for convenience
using MatchedPairs = vector<pair<Vertex*, Vertex*>>;

// Define infinity as a very large value
const double INF = numeric_limits<double>::max();

Graph combineMSTAndMWPM(const Graph* MST, const vector<Vertex*>& oddDegreeVertices, unordered_map<Vertex*, unordered_map<Vertex*, double>>& shortestPaths) {
    Graph multigraph = *MST;

    // Find the minimum weight perfect matching using the Hungarian algorithm
    MatchedPairs mwpm = hungarianAlgorithm(oddDegreeVertices, shortestPaths);

    // Add edges from MWPM to the multigraph
    for (const auto& pair : mwpm) {
        Vertex* u = pair.first;
        Vertex* v = pair.second;
        double weight = shortestPaths[u][v];
        multigraph.addEdge(u->getInfo(), v->getInfo(), weight);
    }

    return multigraph;
}

// Helper function to find an augmenting path
bool findAugmentingPath(Vertex* u, unordered_map<Vertex*, bool>& visited, unordered_map<Vertex*, Vertex*>& match, unordered_map<Vertex*, unordered_map<Vertex*, double>>& weights) {
    for (Edge* edge : u->getAdj()) {
        Vertex* v = edge->getDest();
        if (!visited[v]) {
            visited[v] = true;
            if (match[v] == nullptr || findAugmentingPath(match[v], visited, match, weights)) {
                match[v] = u;
                return true;
            }
        }
    }
    return false;
}
// Hungarian algorithm function
MatchedPairs hungarianAlgorithm(const vector<Vertex*>& oddDegreeVertices, unordered_map<Vertex*, unordered_map<Vertex*, double>>& shortestPaths) {
    MatchedPairs result;

    unordered_map<Vertex*, Vertex*> match;
    for (Vertex* u : oddDegreeVertices) {
        match[u] = nullptr;
    }

    for (Vertex* u : oddDegreeVertices) {
        unordered_map<Vertex*, bool> visited;
        for (Vertex* v : oddDegreeVertices) {
            visited[v] = false;
        }
        findAugmentingPath(u, visited, match, shortestPaths);
    }

    for (const auto& pair : match) {
        if (pair.second != nullptr) {
            result.push_back({pair.second, pair.first});
        }
    }

    return result;
}

void findEulerianWalkDFS(Vertex* u, Graph* multigraph, std::vector<Vertex*>& walk) {
    while (!u->getAdj().empty()) {
        Edge* edge = u->getAdj().front();
        Vertex* v = edge->getDest();
        u->removeEdge(edge->getDest()->getInfo()); // Remove the edge from the multigraph
        findEulerianWalkDFS(v, multigraph, walk);
    }
    walk.push_back(u);
}

std::vector<Vertex*> findEulerianWalk(Graph* multigraph) {
    std::vector<Vertex*> walk;

    // Check if the graph is empty
    if (multigraph->getVertexMap().empty()) {
        return walk;
    }

    // Initialize visited map
    std::unordered_map<std::string, bool> visited;
    for (const auto& pair : multigraph->getVertexMap()) {
        visited[pair.first] = false;
    }

    // Start the walk from an arbitrary vertex
    Vertex* startVertex = multigraph->getVertexMap().begin()->second;

    // Perform DFS to find the Eulerian walk
    findEulerianWalkDFS(startVertex, multigraph, walk);

    // Reverse the walk to get the correct order
    std::reverse(walk.begin(), walk.end());

    return walk;
}


std::vector<Vertex*> substituteShortestPath(const std::vector<Vertex*>& eulerianCircuit, Graph* graph) {
    std::vector<Vertex*> tspTour;

    for (size_t i = 0; i < eulerianCircuit.size() - 1; ++i) {
        Vertex* currentVertex = eulerianCircuit[i];
        Vertex* nextVertex = eulerianCircuit[i + 1];

        std::vector<Vertex*> shortestPath = findShortestPath(currentVertex, nextVertex, graph);

        tspTour.insert(tspTour.end(), shortestPath.begin(), shortestPath.end() - 1);
    }

    tspTour.push_back(eulerianCircuit.back());  // Add the last vertex from the Eulerian circuit

    return tspTour;
}



std::vector<Vertex*> findShortestPath(Vertex* start, Vertex* end, Graph* graph) {
    // Initialize data structures for storing distances and previous vertices
    std::unordered_map<Vertex *, double> distances;
    std::unordered_map<Vertex *, Vertex *> previous;
    std::priority_queue<std::pair<double, Vertex *>, std::vector<std::pair<double, Vertex *>>, std::greater<>> pq;

    // Initialize distances to infinity and add the start vertex to the priority queue
    for (const auto &pair: graph->getVertexMap()) {
        distances[pair.second] = std::numeric_limits<double>::infinity();
        previous[pair.second] = nullptr;
    }
    distances[start] = 0;
    pq.emplace(0, start);

// Dijkstra's algorithm
    while (!pq.empty()) {
        Vertex *current = pq.top().second;
        pq.pop();

        // Stop if the destination vertex is reached
        if (current == end) {
            break;
        }

        // Iterate over adjacent vertices
        for (Edge *edge: current->getAdj()) {
            Vertex *neighbor = edge->getDest();
            double weight = edge->getWeight();
            double newDistance = distances[current] + weight;

            // Relaxation step
            if (newDistance < distances[neighbor]) {
                distances[neighbor] = newDistance;
                previous[neighbor] = current;
                pq.emplace(newDistance, neighbor);
            }
        }
    }

// Reconstruct the shortest path
    std::vector<Vertex *> shortestPath;
    Vertex *current = end;
    while (current != nullptr) {
        shortestPath.push_back(current);
        current = previous[current];
    }
    std::reverse(shortestPath.begin(), shortestPath.end());

    return shortestPath;
}

double TSPExtendedChristofides(Graph* graph, const string& startVertexLabel) {
    // Step 1: Floyd Warshall Algorithm
    unordered_map<Vertex*, unordered_map<Vertex*, double>> shortestPaths;
    floydWarshall(graph, shortestPaths);

    // Step 2: Minimum Spanning Tree (MST)
    Graph MST = primMST(graph, startVertexLabel);

    // Step 3: Find Odd Degree Vertices
    vector<Vertex*> oddDegreeVertices = findOddDegreeVertices(&MST);

    // Step 4: Combine MST and MWPM
    Graph multigraph = combineMSTAndMWPM(&MST, oddDegreeVertices, shortestPaths);

    // Step 5: Find Eulerian Walk
    vector<Vertex*> eulerianWalk = findEulerianWalk(&multigraph);

    // Step 6: Substitute Shortest Paths
    vector<Vertex*> tspTour = substituteShortestPath(eulerianWalk, graph);

    // Calculate total cost of TSP tour
    double totalCost = calculateTotalCost(tspTour, graph);

    return totalCost;
}