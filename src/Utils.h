#ifndef PROJETO_2_UTILS_H
#define PROJETO_2_UTILS_H
#include <cmath>
#include <vector>
#include "Graph.h"

struct CompareWeight {
    bool operator()(const Edge* a, const Edge* b) const {
        return a->getWeight() > b->getWeight();
    }
};

double haversineDistance(double la1, double lo1, double la2, double lo2);

#endif //PROJETO_2_UTILS_H
