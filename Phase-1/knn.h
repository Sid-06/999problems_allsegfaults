#include <vector>
#include <string>


#pragma once
#include <string>

struct KNN {
    int id;
    std::string poi;
    double lat, lon;
    int k;
    std::string metric;
};
