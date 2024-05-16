#include <algorithm>
#include "graph.h"

Vertex::Vertex(const std::string& in): info(in) {}

Edge::Edge(Vertex *s, Vertex *d, double w): src(s), dest(d), weight(w) {}

std::string Vertex::getInfo() const {
    return info;
}

void Vertex::setInfo(const std::string& in) {
    info = in;
}

Edge* Vertex::getPath() const {
    return path;
}

void Vertex::setPath(Edge* path) {
    this->path = path;
}

Vertex* Edge::getDest() const {
    return dest;
}

Vertex* Edge::getSource() const {
    return src;
}

double Edge::getWeight() const {
    return weight;
}

double Edge::getFlow() const {
    return this->flow;
}

void Edge::setFlow(double flow) {
    this->flow = flow;
}

Vertex* Graph::findVertex(const std::string& in) const {
    auto it = vertexMap.find(in);
    if (it != vertexMap.end()) {
        return it->second;
    }
    return nullptr;
}

std::unordered_map<std::string, Vertex*> Graph::getVertexMap() {
    return vertexMap;
}

bool Vertex::isVisited() const {
    return visited;
}

void Vertex::setVisited(bool v) {
    visited = v;
}

std::vector<Edge*> Vertex::getAdj() const {
    return adj;
}

bool Graph::addVertex(const std::string& in) {
    if (findVertex(in) != nullptr)
        return false;
    auto* v = new Vertex(in);
    vertexMap[in] = v;
    return true;
}

bool Graph::addEdge(const std::string& source, const std::string& dest, double weight) {
    auto v1 = findVertex(source);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, weight);
    return true;
}

Edge* Vertex::addEdge(Vertex* dest, double weight) {
    Edge* edge = new Edge(this,dest, weight);
    adj.push_back(edge);
    dest->incoming.push_back(edge);
    return edge;
}

void Vertex::deleteEdge(Edge *edge) {
    Vertex *dest = edge->getDest();
    // Remove the corresponding edge from the incoming list
    auto it = dest->incoming.begin();
    while (it != dest->incoming.end()) {
        if ((*it)->getSource()->getInfo() == info) {
            it = dest->incoming.erase(it);
        }
        else {
            it++;
        }
    }
    delete edge;
}

bool Vertex::isProcessing() const {
    return this->processing;
}

unsigned int Vertex::getIndegree() const {
    return this->indegree;
}

double Vertex::getDist() const {
    return this->dist;
}

vector<Edge *> Vertex::getIncoming() const {
    return this->incoming;
}

void Vertex::setDist(double dist) {
    this->dist = dist;
}

void Vertex::setIndegree(unsigned int indegree) {
    this->indegree = indegree;
}

void Vertex::setProcessing(bool processing) {
    this->processing = processing;
}

bool Vertex::removeEdge(string info) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge *edge = *it;
        Vertex *dest = edge->getDest();
        if (dest->getInfo() == info) {
            it = adj.erase(it);
            deleteEdge(edge);
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        }
        else {
            it++;
        }
    }
    return removedEdge;
}

void Vertex::removeOutgoingEdges() {
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge *edge = *it;
        it = adj.erase(it);
        deleteEdge(edge);
    }
}

void Vertex::setLabel(string label) {
    this->label = label;
}

string Vertex::getLabel() {
    return label;
}

void Vertex::setLongitude(double longitude) {
    this->longitude = longitude;
}

double Vertex::getLongitude() {
    return this->longitude;
}

double Vertex::getLatitude() {
    return this->latitude;
}

void Vertex::setLatitude(double latitude) {
    this->latitude = latitude;
}

bool Graph::removeVertex(const std::string& in) {
    auto it = vertexMap.find(in);
    if (it == vertexMap.end())
        return false;

    Vertex* v = it->second;
    v->removeOutgoingEdges();
    vertexMap.erase(it);
    delete v;
    return true;
}


void Graph::dfsVisit(Vertex* v, std::vector<std::string>& res) const {
    v->visited = true;
    res.push_back(v->info);
    for (auto& e : v->adj) {
        auto w = e->dest;
        if (!w->visited)
            dfsVisit(w, res);
    }
}

bool Graph::removeEdge(const string &source, const string &dest) {
    Vertex * srcVertex = findVertex(source);
    if (srcVertex == nullptr) {
        return false;
    }
    return srcVertex->removeEdge(dest);
}

int Graph::getNumVertex() const {
    return vertexMap.size();
}

int Graph::getNumEdges() const {
    int numEdges = 0;
    for (const auto& pair : vertexMap) {
        Vertex* vertex = pair.second;
        for (const auto& neighbor : vertex->getAdj()) {
            if (vertex->getInfo() < neighbor->getDest()->getInfo()) {
                ++numEdges;
            }
        }
    }
    return numEdges;
}


Edge* Graph::findEdge(const std::string& source, const std::string& dest) const {
    Vertex* srcVertex = findVertex(source);
    Vertex* destVertex = findVertex(dest);
    if (srcVertex == nullptr || destVertex == nullptr)
        return nullptr;

    for (Edge* edge : srcVertex->getAdj()) {
        if (edge->getDest() == destVertex)
            return edge;
    }

    return nullptr;
}
