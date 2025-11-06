#include <string>

struct KSP_E{
    int id;
    int source;
    int target;
    int k;
    std::string type;
    std::string mode;
};



struct KSP_H{
    int id;
    int source;
    int target;
    int k;
    std::string type;
    int overlap_threshold;
};