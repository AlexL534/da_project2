/**
 * @file Graph.h
 * @brief This file contains the definition of nodes, edges and graph
 */

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
using namespace std;

class Edge;
class Graph;
class Vertex;

/**
 * @brief Represents a vertex in the graph.
 */
class Vertex {
    string info;                        ///< Information or identifier of the vertex
    vector<Edge *> adj;                 ///< Outgoing edges
    bool visited = false;               ///< Visitation state
    bool processing = false;            ///< Processing state
    Edge* path = nullptr;               ///< Pointer to the edge leading to this vertex
    unsigned int indegree;              ///< In-degree of the vertex
    double dist = 0;                    ///< Distance value
    vector<Edge *> incoming;            ///< Incoming edges
    string label;                       ///< Label for the vertex
    double longitude;                   ///< Longitude coordinate
    double latitude;                    ///< Latitude coordinate

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

    /**
    * @brief Gets the information/ID of the vertex.
    * Time Complexity: O(1)
    * @return The information/ID of the vertex.
    */
    string getInfo() const;

     /**
     * @brief Gets the outgoing edges of the vertex.
     * Time Complexity: O(1)
     * @return A vector of outgoing edges.
     */
    vector<Edge*> getAdj() const;

    /**
    * @brief Checks if the vertex has been visited.
    * Time Complexity: O(1)
    * @return True if the vertex has been visited, false otherwise.
    */
    bool isVisited() const;

    /**
     * @brief Checks if the vertex is currently being processed.
     * Time Complexity: O(1)
     * @return True if the vertex is being processed, false otherwise.
     */
    bool isProcessing() const;

    /**
     * @brief Gets the in-degree of the vertex.
     * Time Complexity: O(1)
     * @return The in-degree of the vertex.
     */
    unsigned int getIndegree() const;

    /**
     * @brief Gets the distance value of the vertex.
     * Time Complexity: O(1)
     * @return The distance value.
     */
    double getDist() const;

     /**
     * @brief Gets the edge leading to this vertex.
     * Time Complexity: O(1)
     * @return A pointer to the edge leading to this vertex.
     */
    Edge* getPath() const;

    /**
     * @brief Gets the incoming edges of the vertex.
     * Time Complexity: O(1)
     * @return A vector of incoming edges.
     */
    vector<Edge*> getIncoming() const;

    /**
     * @brief Sets the information of the vertex.
     * Time Complexity: O(1)
     * @param in The new information or identifier.
     */
    void setInfo(const string& in);

    /**
     * @brief Sets the visited state of the vertex.
     * Time Complexity: O(1)
     * @param visited The new visited state.
     */
    void setVisited(bool visited);

    /**
     * @brief Sets the processing state of the vertex.
     * Time Complexity: O(1)
     * @param processing The new processing state.
     */
    void setProcessing(bool processing);

    /**
     * @brief Sets the in-degree of the vertex.
     * Time Complexity: O(1)
     * @param indegree The new in-degree.
     */
    void setIndegree(unsigned int indegree);

    /**
     * @brief Sets the distance value of the vertex.
     * Time Complexity: O(1)
     * @param dist The new distance value.
     */
    void setDist(double dist);

    /**
     * @brief Sets the edge leading to this vertex.
     * Time Complexity: O(1)
     * @param path The new edge leading to this vertex.
     */
    void setPath(Edge* path);

    /**
     * @brief Sets the label of the vertex.
     * Time Complexity: O(1)
     * @param label The new label.
     */
    void setLabel(string label);

    /**
     * @brief Gets the label of the vertex.
     * Time Complexity: O(1)
     * @return The label of the vertex.
     */
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

    /**
     * @brief Sets the longitude of the vertex.
     * Time Complexity: O(1)
     * @param longitude The new longitude.
     */
    void setLongitude(double longitude);

    /**
     * @brief Gets the longitude of the vertex.
     * Time Complexity: O(1)
     * @return The longitude.
     */
    double getLongitude();

    /**
     * @brief Sets the latitude of the vertex.
     * Time Complexity: O(1)
     * @param latitude The new latitude.
     */
    void setLatitude(double latitude);

    /**
     * @brief Gets the latitude of the vertex.
     * Time Complexity: O(1)
     * @return The latitude.
     */
    double getLatitude();

    friend class Graph;
};

/**
 * @brief Represents an edge in the graph.
 */
class Edge {
    Vertex * src;         ///< Source vertex of the edge
    Vertex * dest;        ///< Destination vertex of the edge
    double flow;          ///< Flow through the edge
    double weight;        ///< Weight of the edge
public:
    /**
     * @brief Constructs an edge with the given source, destination, and weight.
     * Time Complexity: O(1)
     * @param src The source vertex.
     * @param dest The destination vertex.
     * @param weight The weight of the edge.
     */
    Edge(Vertex *src, Vertex *dest, double weight);

    /**
     * @brief Gets the destination vertex of the edge.
     * Time Complexity: O(1)
     * @return A pointer to the destination vertex.
     */
    Vertex *getDest() const;

    /**
     * @brief Gets the source vertex of the edge.
     * Time Complexity: O(1)
     * @return A pointer to the source vertex.
     */
    Vertex *getSource() const;

    /**
     * @brief Gets the flow through the edge.
     * Time Complexity: O(1)
     * @return The flow through the edge.
     */
    double getFlow() const;

    /**
     * @brief Gets the weight of the edge.
     * Time Complexity: O(1)
     * @return The weight of the edge.
     */
    double getWeight() const;

    /**
     * @brief Sets the flow through the edge.
     * Time Complexity: O(1)
     * @param flow The new flow value.
     */
    void setFlow(double flow);

    friend class Graph;
    friend class Vertex;
};

/**
 * @brief Represents a graph.
 */
class Graph {
    std::unordered_map<std::string, Vertex *> vertexMap; ///< Mapping of vertex identifiers to Vertex objects
public:
    Graph() = default;

    /**
     * @brief Finds a vertex in the graph by its info/ID.
     * Time Complexity: O(1)
     * @param in The info/ID of the vertex to find.
     * @return A pointer to the vertex if found, otherwise `nullptr`.
     */
    Vertex *findVertex(const string &in) const;

    /**
     * @brief Adds a vertex to the graph.
     * Time Complexity: O(1)
     * @param in The info/ID of the vertex to add.
     * @return True if the vertex was added, false if it already exists.
     */
    bool addVertex(const string &in);

    /**
     * @brief Removes a vertex from the graph.
     * Time Complexity: O(E), where E is the number of outgoing edges from the vertex to be removed.
     * @param in The info/id of the vertex to remove.
     * @return True if the vertex was removed, false if it does not exist.
     */
    bool removeVertex(const string &in);

    /**
     * @brief Adds an edge between two vertices in the graph.
     * Time Complexity: O(1)
     * @param source The info/ID of the source vertex.
     * @param dest The info/ID of the destination vertex.
     * @param weight The weight of the edge to be added.
     * @return True if the edge was successfully added or updated, false otherwise.
     */
    bool addEdge(const string &source, const string &dest, double weight);

    /**
     * @brief Removes an edge between two vertices in the graph.
     * Time Complexity: O(E), where E is the number of outgoing edges from the source vertex.
     * @param source The info/ID of the source vertex.
     * @param dest The info/ID of the destination vertex.
     * @return True if the edge was successfully removed, false otherwise.
     */
    bool removeEdge(const string &source, const string &dest);

    /**
     * @brief Gets the number of vertices in the graph.
     * Time Complexity: O(1).
     * @return The number of vertices in the graph.
     */
    int getNumVertex() const;

    /**
     * @brief Gets a mapping of vertex identifiers to Vertex objects.
     * Time Complexity: O(1).
     * @return An unordered map containing the mapping of vertex identifiers to Vertex objects.
     */
    std::unordered_map<std::string, Vertex*> getVertexMap();

    /**
     * @brief Performs a depth-first search (DFS) traversal starting from the specified vertex.
     * Time Complexity: O(V + E), where V is the number of vertices and E is the number of edges in the connected component containing the starting vertex.
     * @param v A pointer to the starting vertex for the DFS traversal.
     * @param res A vector to store the traversal result (IDs of visited vertices).
     */
    void dfsVisit(Vertex *v, vector<string>& res) const;

    /**
     * @brief Gets the number of edges in the graph.
     * Time Complexity: O(V + E), where V is the number of vertices and E is the number of edges.
     * @return The number of edges in the graph.
     */
    int getNumEdges() const;

    /**
     * @brief Finds an edge between two vertices in the graph.
     * Time Complexity: O(E), where E is the number of outgoing edges from the source vertex.
     * @param source The info/ID of the source vertex.
     * @param dest The info/ID of the destination vertex.
     * @return A pointer to the edge between the specified vertices, or nullptr if not found.
     */
    Edge* findEdge(const std::string& source, const std::string& dest) const;

    /**
     * @brief Resets the visited flag for all vertices in the graph.
     */
    void resetVisited();
};

#endif // GRAPH_H
