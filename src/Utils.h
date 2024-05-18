#ifndef PROJETO_2_UTILS_H
#define PROJETO_2_UTILS_H
#include <cmath>
#include <vector>
#include "Graph.h"

/**
 * @brief Comparator for edges based on their weight.
 */
struct CompareWeight {
    /**
     * @brief Compares two edges by their weight.
     * @param a Pointer to the first Edge.
     * @param b Pointer to the second Edge.
     * @return True if the weight of the first edge is greater than the weight of the second edge.
     */
    bool operator()(const Edge* a, const Edge* b) const {
        return a->getWeight() > b->getWeight();
    }
};

/**
 * @brief Calculates the Haversine distance between two points
 * @param la1 Latitude of the first point in degrees.
 * @param lo1 Longitude of the first point in degrees.
 * @param la2 Latitude of the second point in degrees.
 * @param lo2 Longitude of the second point in degrees.
 * @return The distance between the two points in meters.
 */
double haversineDistance(double la1, double lo1, double la2, double lo2);

#endif //PROJETO_2_UTILS_H
