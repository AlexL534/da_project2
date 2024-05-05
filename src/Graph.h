#include <cstddef>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <stack>
#include <list>
#include <string>
#include <limits>
#include <queue>
#include <map>
#include <set>
#include <algorithm>

using namespace std;

class Edge;
class Graph;
class Vertex;

class Vertex {
    string info;
    vector<Edge *> adj; //outgoing edges
    bool visited = false;
    bool processing = false;
    Edge* path = nullptr; //pointer to the edge that leads to the current vertex
    unsigned int indegree;
    double dist = 0;
    vector<Edge *> incoming;
    void deleteEdge(Edge* edge);

public:
    Vertex(const string& in);
    string getInfo() const;
    vector<Edge*> getAdj() const;
    bool isVisited() const;
    bool isProcessing() const;
    unsigned int getIndegree() const;
    double getDist() const;
    Edge* getPath() const;
    vector<Edge*> getIncoming() const;
    void setInfo(const string& in);
    void setVisited(bool visited);
    void setProcessing(bool processing);
    void setIndegree(unsigned int indegree);
    void setDist(double dist);
    void setPath(Edge* path);
    Edge* addEdge(Vertex *dest, double weight);
    bool removeEdge(string info);
    void removeOutgoingEdges();

    friend class Graph;
};

class Edge {
    Vertex * src;
    Vertex * dest;
    double flow;
    double weight;
public:
    Edge(Vertex *src, Vertex *dest, int weight);
    Vertex *getDest() const;
    Vertex *getSource() const;
    double getFlow() const;
    double getWeight() const;
    void setFlow(double flow);
    friend class Graph;
    friend class Vertex;
};

class Graph {
    vector<Vertex*> vertexSet;
public:
    Graph() = default;

    Vertex *findVertex(const string &in) const;
    bool addVertex(const string &in);
    bool removeVertex(const string &in);
    bool addEdge(const string &source, const string &dest, double weight);
    bool removeEdge(const string &source, const string &dest);
    int getNumVertex() const;
    vector<Vertex*> getVertexSet() const;
    void dfsVisit(Vertex *v, vector<string>& res) const;
};