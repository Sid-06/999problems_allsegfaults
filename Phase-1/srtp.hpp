#include<unordered_map>
#include<string>

struct SRTP{
    int id;
    int source;
    int target;
    std::string mode;
    std::unordered_map<int,bool> forbidden_nodes;
    std::unordered_map<std::string,bool> forbidden_road_types = {
    {"expressway",false},
    {"primary",false},
    {"secondary",false},
    {"tertiary",false},
    {"local",false}
    };
};