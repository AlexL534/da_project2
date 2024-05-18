#ifndef GRAPH_H
#define GRAPH_H

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
//diferença para o das aulas: vertexSet é armazenado num unordered map e chama-se vertexMap
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
    string label;
    double longitude;
    double latitude;

     /**
     * @brief Deletes an edge from this vertex.
     * Time Complexity: O(N) being N the number of incoming edges for the dest vertex of the given edge
     * @param edge Pointer to the edge to delete.
     */
    void deleteEdge(Edge* edge);

public:
     /**
     * @brief Constructs a vertex with the given information.
     * @param in The info/ID of the vertex.
     */
    Vertex(const string& in);

    string getInfo() const;

     /**
     * @brief Gets the outgoing edges of the vertex.
     * Time Complexity: O(1)
     * @return A vector of outgoing edges.
     */
    vector<Edge*> getAdj() const;

    bool isVisited() const;
    bool isProcessing() const;
    unsigned int getIndegree() const;
    double getDist() const;

     /**
     * @brief Gets the edge leading to this vertex.
     * Time Complexity: O(1)
     * @return A pointer to the edge leading to this vertex.
     */
    Edge* getPath() const;

    vector<Edge*> getIncoming() const;
    void setInfo(const string& in);
    void setVisited(bool visited);
    void setProcessing(bool processing);
    void setIndegree(unsigned int indegree);
    void setDist(double dist);
    void setPath(Edge* path);
    void setLabel(string label);
    string getLabel();

     /**
     * @brief Adds an edge to this vertex.
     * Time Complexity: O(1)
     * @param dest The destination vertex.
     * @param weight The weight of the edge.
     * @return A pointer to the newly added edge.
     */
    Edge* addEdge(Vertex *dest, double weight);

     /**
     * @brief Removes an edge from this vertex.
     * Time Complexity: O(E), where E is the number of outgoing edges.
     * @param info The info/ID of the destination vertex.
     * @return True if an edge was removed, false otherwise.
     */
    bool removeEdge(string info);

     /**
     * @brief Removes all outgoing edges from this vertex.
     * Time Complexity: O(E), where E is the number of outgoing edges.
     */
    void removeOutgoingEdges();

    void setLongitude(double longitude);
    double getLongitude();
    void setLatitude(double latitude);
    double getLatitude();

    friend class Graph;
};

class Edge {
    Vertex * src;
    Vertex * dest;
    double flow;
    double weight;
public:
    Edge(Vertex *src, Vertex *dest, double weight);
    Vertex *getDest() const;
    Vertex *getSource() const;
    double getFlow() const;
    double getWeight() const;
    void setFlow(double flow);
    friend class Graph;
    friend class Vertex;
};

class Graph {
    std::unordered_map<std::string, Vertex *> vertexMap;
public:
    Graph() = default;

    /**
     * @brief Finds a vertex in the graph by its name.
     *
    Time Complexity: O(1) on average, due to the underlying hash map implementation.
     *
     * @param in The info/id of the vertex to find.
     * @return A pointer to the vertex if found, otherwise `nullptr`.
     */
    Vertex *findVertex(const string &in) const;
    bool addVertex(const string &in);
    bool removeVertex(const string &in);
    bool addEdge(const string &source, const string &dest, double weight);
    bool removeEdge(const string &source, const string &dest);
    int getNumVertex() const;
    std::unordered_map<std::string, Vertex*> getVertexMap();
    void dfsVisit(Vertex *v, vector<string>& res) const;
    int getNumEdges() const;
    Edge* findEdge(const std::string& source, const std::string& dest) const;
};

#endif // GRAPH_H
