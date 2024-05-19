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

vector<Vertex*> nearestNeighborTSP(Graph* graph, const string& start) {
    unordered_set<Vertex*> visited;
    vector<Vertex*> path;
    Vertex * current = graph->findVertex(start);
    visited.insert(current);
    path.push_back(current);

    while (visited.size() < graph->getNumVertex()) {
        double minCost = numeric_limits<double>::max();
        Vertex* nextNode;

        for (Edge* edge : current->getAdj()) {
            Vertex *neighbor = edge->getDest();
            if (visited.find(neighbor) == visited.end() && edge->getWeight() < minCost) {
                minCost = edge->getWeight();
                nextNode = neighbor;
            }
        }

        if (nextNode == nullptr) return {};
        visited.insert(nextNode);
        path.push_back(nextNode);
        current = nextNode;
    }

    path.push_back(graph->findVertex(start));
    return path;
}


vector<Vertex*> hybridMSTAndNNTSP(Graph* graph,const string& start) {
    auto mstGraph = primMST(graph, start);
    auto nnPath = nearestNeighborTSP(graph, start);


    return nnPath;
}