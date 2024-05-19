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

//4.4
