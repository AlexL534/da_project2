#include "Utils.h"

double convertToRadians(double coord) {
    return coord * M_PI / 180.0;
}

double haversineDistance(std::vector<double> coord1, std::vector<double> coord2) {
    double rad_lat1 = convertToRadians(coord1[0]);
    double rad_lon1 = convertToRadians(coord1[1]);
    double rad_lat2 = convertToRadians(coord2[0]);
    double rad_lon2 = convertToRadians(coord2[1]);

    double delta_lat = rad_lat2 - rad_lat1;
    double delta_lon = rad_lon2 - rad_lon1;

    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(rad_lat1) * cos(rad_lat2) *
               sin(delta_lon / 2.0) * sin(delta_lon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double earth_radius = 6371000;

    return earth_radius * c;
}