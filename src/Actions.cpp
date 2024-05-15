#include "Actions.h"

double TSPBacktracking(Graph* graph) {
    MemoizationTable memoization;
    Vertex* start = *graph->getVertexSet().begin();
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



double haversineDistance(double la1, double lo1, double la2, double lo2){
    double R = 6371e3;
    double phi1 = la1 * M_PI/180;
    double phi2 = la2 * M_PI/180;
    double deltaPhi = (la2-la1) * M_PI/180;
    double deltaLambda = (lo2-lo1) * M_PI/180;

    double a = sin(deltaPhi/2) * sin(deltaPhi/2) +
            cos(phi1) * cos(phi2) *
            sin(deltaLambda/2) * sin(deltaLambda/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    double d = R * c;
    return d;
}

struct CompareWeight {
    bool operator()(const Edge* a, const Edge* b) const {
        return a->getWeight() > b->getWeight();
    }
};

Graph primMST(Graph* graph, const string& startVertexLabel) {

    priority_queue<Edge*, vector<Edge*>, CompareWeight> pq;

    unordered_map<string, bool> visited;

    for (Vertex* vertex : graph->getVertexSet()) {
        visited[vertex->getInfo()] = false;
    }

    Vertex* startVertex = graph->findVertex(startVertexLabel);
    visited[startVertexLabel] = true;

    for (Edge* edge : startVertex->getAdj()) {
        pq.push(edge);
    }

    Graph MST;

    double minPath = 0;

    while (!pq.empty()) {
        Edge* minEdge = pq.top();
        pq.pop();

        Vertex* src = minEdge->getSource();
        Vertex* dest = minEdge->getDest();

        if (visited[src->getInfo()] && visited[dest->getInfo()]) {
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

    cout << "Minimum Spanning Tree: " << minPath << endl;

    return MST;
}

void preOrderWalk(Vertex* vertex, std::unordered_set<Vertex*>& visited, std::vector<Vertex*>& preOrderList, vector<Edge*>& edges) {
    if (vertex == nullptr) return;

    preOrderList.push_back(vertex);
    visited.insert(vertex);

    for (Edge* edge : vertex->getAdj()) {
        Vertex* nextVertex = edge->getDest();
        if (visited.find(nextVertex) == visited.end()) {
            edges.push_back(edge);
            preOrderWalk(nextVertex, visited, preOrderList, edges);
        }
    }
}
void connectAllEdges(Graph *graph){
    for(auto vertex : graph->getVertexSet()){
        for(auto vertex2 : graph->getVertexSet()){
            Edge* e = graph->findEdge(vertex->getInfo(), vertex2->getInfo());
            if(!e){
                double dist = haversineDistance(vertex->getLatitude(), vertex->getLongitude(), vertex2->getLatitude(), vertex2->getLongitude());
                graph->addEdge(vertex->getInfo(), vertex2->getInfo(), dist);
            }
        }
    }
}
map<vector<Vertex*>, double> TSPTriangularApproximation(Graph* graph) {
    std::vector<Vertex*> tour;
    map<vector<Vertex*>, double> res;
    double minPath = 0;

    connectAllEdges(graph);

    Graph MST = primMST(graph, graph->getVertexSet()[0]->getInfo());
    std::unordered_set<Vertex*> visited;
    std::vector<Vertex*> preOrderList;
    vector<Edge*> edges;
    preOrderWalk(MST.getVertexSet()[0], visited, preOrderList, edges);

    std::unordered_set<Vertex*> visitedInTour;
    for (Vertex* vertex : preOrderList) {
        if (visitedInTour.find(vertex) == visitedInTour.end()) {
            tour.push_back(vertex);
            visitedInTour.insert(vertex);
        }
    }

    Edge* edg = graph->findEdge(graph->getVertexSet().back()->getInfo(), preOrderList[0]->getInfo());
    minPath += edg->getWeight();
    tour.push_back(preOrderList[0]);



    for(auto e: edges){
        minPath += e->getWeight();
    }

    res[tour] = minPath;


    cout << minPath << endl;
    for (auto t : tour){
        cout << t->getInfo() << " ";
    }

    cout << endl;


    return res;
}

