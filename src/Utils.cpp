#include "Utils.h"

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